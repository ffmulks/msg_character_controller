//! Navigation handover system for bevy_northstar integration.
//!
//! This module provides a robust system for handling transitions between
//! grid-based pathfinding (bevy_northstar) and free flight when actors
//! need to navigate to/from positions outside the navigation grid.
//!
//! # Problem
//!
//! bevy_northstar operates on a fixed-size grid. When start or goal positions
//! fall outside grid bounds, pathfinding fails with errors like:
//!
//! ```text
//! Start or goal position is out of bounds. Start: UVec3(101, 334, 0), Goal: UVec3(77, 167, 0)
//! ```
//!
//! # Solution
//!
//! This module provides a state machine that handles all permutations:
//!
//! 1. **Both in grid** → Normal pathfinding (delegated to northstar)
//! 2. **Start in, target out** → Navigate to edge, then fly to target
//! 3. **Start out, target in** → Fly to edge, then navigate to target
//! 4. **Both out** → Direct flight (optionally via edge for obstacle avoidance)
//!
//! # Usage
//!
//! ```rust,ignore
//! use msg_character_controller::navigation::*;
//!
//! // Configure grid bounds (typically derived from your northstar Grid)
//! app.insert_resource(NavigationBounds {
//!     min: Vec2::ZERO,
//!     max: Vec2::new(192.0 * TILE_SIZE, 192.0 * TILE_SIZE),
//!     edge_margin: TILE_SIZE * 2.0, // Safety margin from grid edge
//! });
//!
//! // Add navigation target to an entity
//! commands.entity(actor).insert(NavigationTarget::new(target_pos));
//!
//! // The handover system will automatically manage transitions
//! ```

use bevy::prelude::*;

/// Resource defining the navigable grid bounds.
///
/// This should match your bevy_northstar Grid dimensions, converted to world coordinates.
#[derive(Resource, Clone, Debug)]
pub struct NavigationBounds {
    /// Minimum corner of the navigable grid (world coordinates).
    pub min: Vec2,
    /// Maximum corner of the navigable grid (world coordinates).
    pub max: Vec2,
    /// Safety margin from the grid edge for edge points.
    /// Edge points will be placed this far inside the grid boundary.
    pub edge_margin: f32,
}

impl Default for NavigationBounds {
    fn default() -> Self {
        Self {
            min: Vec2::ZERO,
            max: Vec2::new(1920.0, 1920.0),
            edge_margin: 32.0,
        }
    }
}

impl NavigationBounds {
    /// Create new navigation bounds from grid dimensions and tile size.
    pub fn from_grid(grid_width: u32, grid_height: u32, tile_size: f32) -> Self {
        Self {
            min: Vec2::ZERO,
            max: Vec2::new(grid_width as f32 * tile_size, grid_height as f32 * tile_size),
            edge_margin: tile_size * 2.0,
        }
    }

    /// Create new navigation bounds from grid dimensions, tile size, and origin.
    pub fn from_grid_with_origin(
        grid_width: u32,
        grid_height: u32,
        tile_size: f32,
        origin: Vec2,
    ) -> Self {
        Self {
            min: origin,
            max: origin + Vec2::new(grid_width as f32 * tile_size, grid_height as f32 * tile_size),
            edge_margin: tile_size * 2.0,
        }
    }

    /// Check if a position is inside the navigable grid bounds.
    pub fn contains(&self, pos: Vec2) -> bool {
        pos.x >= self.min.x && pos.x <= self.max.x && pos.y >= self.min.y && pos.y <= self.max.y
    }

    /// Check if a position is inside the bounds with margin applied.
    /// This is more conservative - positions must be `edge_margin` inside the bounds.
    pub fn contains_with_margin(&self, pos: Vec2) -> bool {
        pos.x >= self.min.x + self.edge_margin
            && pos.x <= self.max.x - self.edge_margin
            && pos.y >= self.min.y + self.edge_margin
            && pos.y <= self.max.y - self.edge_margin
    }

    /// Find the closest point on the grid boundary (with margin applied).
    ///
    /// This is used to find where an actor should enter/exit the grid
    /// when transitioning between free flight and pathfinding.
    pub fn closest_edge_point(&self, from: Vec2, toward: Vec2) -> Vec2 {
        let inner_min = self.min + Vec2::splat(self.edge_margin);
        let inner_max = self.max - Vec2::splat(self.edge_margin);

        // If the target is inside, find where the ray from outside enters
        // If the target is outside, find where the ray from inside exits
        let dir = (toward - from).normalize_or_zero();

        if dir == Vec2::ZERO {
            // No direction, just clamp to nearest edge
            return Vec2::new(
                from.x.clamp(inner_min.x, inner_max.x),
                from.y.clamp(inner_min.y, inner_max.y),
            );
        }

        // Find intersection with all 4 edges and pick the valid one
        let mut best_point = None;
        let mut best_dist = f32::INFINITY;

        // For a point outside heading toward inside (or vice versa),
        // we want the first intersection point along the ray
        let edges = [
            (inner_min.x, true),  // left edge (x = inner_min.x)
            (inner_max.x, true),  // right edge (x = inner_max.x)
            (inner_min.y, false), // bottom edge (y = inner_min.y)
            (inner_max.y, false), // top edge (y = inner_max.y)
        ];

        for (edge_val, is_vertical) in edges {
            let t = if is_vertical {
                if dir.x.abs() < 1e-6 {
                    continue;
                }
                (edge_val - from.x) / dir.x
            } else {
                if dir.y.abs() < 1e-6 {
                    continue;
                }
                (edge_val - from.y) / dir.y
            };

            if t < 0.0 {
                continue; // Behind us
            }

            let point = from + dir * t;

            // Check if point is within the edge bounds
            let valid = if is_vertical {
                point.y >= inner_min.y && point.y <= inner_max.y
            } else {
                point.x >= inner_min.x && point.x <= inner_max.x
            };

            if valid && t < best_dist {
                best_dist = t;
                best_point = Some(point);
            }
        }

        best_point.unwrap_or_else(|| {
            // Fallback: clamp to the nearest corner
            Vec2::new(
                from.x.clamp(inner_min.x, inner_max.x),
                from.y.clamp(inner_min.y, inner_max.y),
            )
        })
    }

    /// Find the closest point on the grid boundary from any position.
    /// Unlike `closest_edge_point`, this doesn't consider direction.
    pub fn nearest_boundary_point(&self, pos: Vec2) -> Vec2 {
        let inner_min = self.min + Vec2::splat(self.edge_margin);
        let inner_max = self.max - Vec2::splat(self.edge_margin);

        // If inside, find closest edge
        if self.contains_with_margin(pos) {
            let dist_left = pos.x - inner_min.x;
            let dist_right = inner_max.x - pos.x;
            let dist_bottom = pos.y - inner_min.y;
            let dist_top = inner_max.y - pos.y;

            let min_dist = dist_left.min(dist_right).min(dist_bottom).min(dist_top);

            if min_dist == dist_left {
                return Vec2::new(inner_min.x, pos.y);
            } else if min_dist == dist_right {
                return Vec2::new(inner_max.x, pos.y);
            } else if min_dist == dist_bottom {
                return Vec2::new(pos.x, inner_min.y);
            } else {
                return Vec2::new(pos.x, inner_max.y);
            }
        }

        // If outside, clamp to boundary
        Vec2::new(
            pos.x.clamp(inner_min.x, inner_max.x),
            pos.y.clamp(inner_min.y, inner_max.y),
        )
    }
}

/// Navigation state machine for handling grid/free-flight transitions.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
pub enum NavigationState {
    /// No active navigation target.
    #[default]
    Idle,

    /// Following a bevy_northstar path (both positions in grid).
    FollowingPath,

    /// Flying toward the grid edge to begin pathfinding.
    /// Used when start is outside grid, target is inside.
    FlyingToEdge,

    /// Flying from grid edge to final target outside grid.
    /// Used when start is inside grid, target is outside.
    FlyingFromEdge,

    /// Direct flight between two out-of-grid positions.
    DirectFlight,

    /// Reached the destination.
    Arrived,
}

/// Describes the scenario based on start/target positions relative to grid.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
pub enum NavigationScenario {
    /// Both start and target are inside the navigable grid.
    #[default]
    BothInside,
    /// Start is inside, target is outside the grid.
    StartInsideTargetOutside,
    /// Start is outside, target is inside the grid.
    StartOutsideTargetInside,
    /// Both start and target are outside the grid.
    BothOutside,
}

impl NavigationScenario {
    /// Determine the scenario from positions and bounds.
    pub fn from_positions(start: Vec2, target: Vec2, bounds: &NavigationBounds) -> Self {
        let start_inside = bounds.contains(start);
        let target_inside = bounds.contains(target);

        match (start_inside, target_inside) {
            (true, true) => Self::BothInside,
            (true, false) => Self::StartInsideTargetOutside,
            (false, true) => Self::StartOutsideTargetInside,
            (false, false) => Self::BothOutside,
        }
    }
}

/// Component requesting navigation to a target position.
///
/// Add this to an entity to trigger navigation. The handover system
/// will automatically manage transitions between pathfinding and free flight.
#[derive(Component, Clone, Debug, Reflect)]
pub struct NavigationTarget {
    /// The final destination in world coordinates.
    pub target: Vec2,
    /// Tolerance radius for considering arrival.
    pub arrival_tolerance: f32,
    /// Speed multiplier for free flight (0.0 to 1.0).
    pub flight_speed: f32,
}

impl NavigationTarget {
    /// Create a new navigation target with default settings.
    pub fn new(target: Vec2) -> Self {
        Self {
            target,
            arrival_tolerance: 16.0,
            flight_speed: 1.0,
        }
    }

    /// Create a navigation target with custom tolerance.
    pub fn with_tolerance(target: Vec2, tolerance: f32) -> Self {
        Self {
            target,
            arrival_tolerance: tolerance,
            flight_speed: 1.0,
        }
    }

    /// Set the flight speed multiplier.
    pub fn with_flight_speed(mut self, speed: f32) -> Self {
        self.flight_speed = speed.clamp(0.0, 1.0);
        self
    }
}

/// Component tracking the current navigation handover state.
///
/// This is automatically added and managed by the navigation systems.
#[derive(Component, Clone, Debug, Default, Reflect)]
pub struct NavigationHandover {
    /// Current state of the navigation state machine.
    pub state: NavigationState,
    /// The intermediate edge point (if transitioning at grid boundary).
    pub edge_point: Option<Vec2>,
    /// The final target position.
    pub final_target: Vec2,
    /// The scenario determined when navigation started.
    pub scenario: Option<NavigationScenario>,
}

impl NavigationHandover {
    /// Create a new handover state for a target.
    pub fn new(target: Vec2) -> Self {
        Self {
            state: NavigationState::Idle,
            edge_point: None,
            final_target: target,
            scenario: None,
        }
    }

    /// Reset to idle state.
    pub fn reset(&mut self) {
        self.state = NavigationState::Idle;
        self.edge_point = None;
        self.scenario = None;
    }

    /// Check if navigation is complete.
    pub fn is_arrived(&self) -> bool {
        self.state == NavigationState::Arrived
    }

    /// Check if currently in free flight mode.
    pub fn is_flying(&self) -> bool {
        matches!(
            self.state,
            NavigationState::FlyingToEdge
                | NavigationState::FlyingFromEdge
                | NavigationState::DirectFlight
        )
    }

    /// Check if currently following a path.
    pub fn is_following_path(&self) -> bool {
        self.state == NavigationState::FollowingPath
    }
}

/// Event fired when navigation state changes.
#[derive(Event, Clone, Debug)]
pub struct NavigationStateChanged {
    pub entity: Entity,
    pub old_state: NavigationState,
    pub new_state: NavigationState,
}

/// Event fired when an entity should request a path from bevy_northstar.
///
/// Listen for this event to trigger `PathRequest` creation in northstar.
/// The `start` and `goal` are guaranteed to be within grid bounds.
#[derive(Event, Clone, Debug)]
pub struct RequestPathfinding {
    pub entity: Entity,
    /// Start position (guaranteed to be in grid bounds).
    pub start: Vec2,
    /// Goal position (guaranteed to be in grid bounds).
    pub goal: Vec2,
}

/// Event fired when free flight should begin.
#[derive(Event, Clone, Debug)]
pub struct BeginFreeFlight {
    pub entity: Entity,
    /// Position to fly toward.
    pub target: Vec2,
    /// Speed multiplier.
    pub speed: f32,
}

/// Event fired when navigation is complete.
#[derive(Event, Clone, Debug)]
pub struct NavigationComplete {
    pub entity: Entity,
    /// Final position reached.
    pub position: Vec2,
}

/// System set for navigation handover systems.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NavigationHandoverSet {
    /// Evaluate navigation targets and determine scenarios.
    Evaluate,
    /// Update state machine based on current position.
    UpdateState,
    /// Apply movement for free flight.
    ApplyMovement,
}

/// Plugin for the navigation handover system.
pub struct NavigationHandoverPlugin;

impl Plugin for NavigationHandoverPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<NavigationTarget>();
        app.register_type::<NavigationHandover>();
        app.register_type::<NavigationState>();

        app.init_resource::<NavigationBounds>();

        app.add_event::<NavigationStateChanged>();
        app.add_event::<RequestPathfinding>();
        app.add_event::<BeginFreeFlight>();
        app.add_event::<NavigationComplete>();

        app.configure_sets(
            FixedUpdate,
            (
                NavigationHandoverSet::Evaluate,
                NavigationHandoverSet::UpdateState,
                NavigationHandoverSet::ApplyMovement,
            )
                .chain(),
        );

        app.add_systems(
            FixedUpdate,
            (
                init_navigation_handover,
                evaluate_navigation_scenario,
            )
                .chain()
                .in_set(NavigationHandoverSet::Evaluate),
        );

        app.add_systems(
            FixedUpdate,
            update_navigation_state.in_set(NavigationHandoverSet::UpdateState),
        );

        app.add_systems(
            FixedUpdate,
            apply_free_flight_movement.in_set(NavigationHandoverSet::ApplyMovement),
        );

        app.add_systems(
            FixedUpdate,
            cleanup_completed_navigation.after(NavigationHandoverSet::ApplyMovement),
        );
    }
}

/// Initialize NavigationHandover for new NavigationTarget components.
fn init_navigation_handover(
    mut commands: Commands,
    query: Query<(Entity, &NavigationTarget), Without<NavigationHandover>>,
) {
    for (entity, target) in &query {
        commands
            .entity(entity)
            .insert(NavigationHandover::new(target.target));
    }
}

/// Evaluate the navigation scenario and set initial state.
fn evaluate_navigation_scenario(
    bounds: Res<NavigationBounds>,
    mut query: Query<
        (Entity, &Transform, &NavigationTarget, &mut NavigationHandover),
        Changed<NavigationTarget>,
    >,
    mut state_events: EventWriter<NavigationStateChanged>,
    mut pathfinding_events: EventWriter<RequestPathfinding>,
    mut flight_events: EventWriter<BeginFreeFlight>,
) {
    for (entity, transform, nav_target, mut handover) in &mut query {
        let start = transform.translation.truncate();
        let target = nav_target.target;

        let scenario = NavigationScenario::from_positions(start, target, &bounds);
        handover.scenario = Some(scenario);
        handover.final_target = target;

        let old_state = handover.state;

        match scenario {
            NavigationScenario::BothInside => {
                // Normal pathfinding - delegate entirely to northstar
                handover.state = NavigationState::FollowingPath;
                handover.edge_point = None;

                pathfinding_events.write(RequestPathfinding {
                    entity,
                    start,
                    goal: target,
                });
            }
            NavigationScenario::StartInsideTargetOutside => {
                // Navigate to edge, then fly out
                let edge = bounds.closest_edge_point(start, target);
                handover.edge_point = Some(edge);
                handover.state = NavigationState::FollowingPath;

                // First, pathfind to the edge
                pathfinding_events.write(RequestPathfinding {
                    entity,
                    start,
                    goal: edge,
                });
            }
            NavigationScenario::StartOutsideTargetInside => {
                // Fly to edge, then pathfind
                let edge = bounds.closest_edge_point(start, target);
                handover.edge_point = Some(edge);
                handover.state = NavigationState::FlyingToEdge;

                flight_events.write(BeginFreeFlight {
                    entity,
                    target: edge,
                    speed: nav_target.flight_speed,
                });
            }
            NavigationScenario::BothOutside => {
                // Direct flight
                handover.edge_point = None;
                handover.state = NavigationState::DirectFlight;

                flight_events.write(BeginFreeFlight {
                    entity,
                    target,
                    speed: nav_target.flight_speed,
                });
            }
        }

        if old_state != handover.state {
            state_events.write(NavigationStateChanged {
                entity,
                old_state,
                new_state: handover.state,
            });
        }
    }
}

/// Update navigation state based on current position and progress.
fn update_navigation_state(
    mut query: Query<(
        Entity,
        &Transform,
        &NavigationTarget,
        &mut NavigationHandover,
    )>,
    mut state_events: EventWriter<NavigationStateChanged>,
    mut pathfinding_events: EventWriter<RequestPathfinding>,
    mut flight_events: EventWriter<BeginFreeFlight>,
    mut complete_events: EventWriter<NavigationComplete>,
) {
    for (entity, transform, nav_target, mut handover) in &mut query {
        let pos = transform.translation.truncate();
        let old_state = handover.state;

        match handover.state {
            NavigationState::Idle => {
                // Nothing to do
            }
            NavigationState::FollowingPath => {
                // Check if we need to transition to flight after reaching edge
                if let Some(edge) = handover.edge_point {
                    if pos.distance(edge) < nav_target.arrival_tolerance {
                        // Reached edge, now fly to final target
                        handover.state = NavigationState::FlyingFromEdge;

                        flight_events.write(BeginFreeFlight {
                            entity,
                            target: handover.final_target,
                            speed: nav_target.flight_speed,
                        });
                    }
                }
                // Otherwise, northstar handles path following
            }
            NavigationState::FlyingToEdge => {
                if let Some(edge) = handover.edge_point {
                    if pos.distance(edge) < nav_target.arrival_tolerance {
                        // Reached edge, now pathfind to target
                        handover.state = NavigationState::FollowingPath;

                        pathfinding_events.write(RequestPathfinding {
                            entity,
                            start: pos,
                            goal: handover.final_target,
                        });
                    }
                }
            }
            NavigationState::FlyingFromEdge | NavigationState::DirectFlight => {
                if pos.distance(handover.final_target) < nav_target.arrival_tolerance {
                    handover.state = NavigationState::Arrived;

                    complete_events.write(NavigationComplete {
                        entity,
                        position: pos,
                    });
                }
            }
            NavigationState::Arrived => {
                // Navigation complete, state persists until target is removed
            }
        }

        if old_state != handover.state {
            state_events.write(NavigationStateChanged {
                entity,
                old_state,
                new_state: handover.state,
            });
        }
    }
}

/// Apply movement intent for free flight navigation.
fn apply_free_flight_movement(
    mut query: Query<(
        &Transform,
        &NavigationTarget,
        &NavigationHandover,
        &mut crate::intent::MovementIntent,
    )>,
) {
    for (transform, nav_target, handover, mut intent) in &mut query {
        if !handover.is_flying() {
            continue;
        }

        let pos = transform.translation.truncate();
        let target = match handover.state {
            NavigationState::FlyingToEdge => handover.edge_point.unwrap_or(handover.final_target),
            NavigationState::FlyingFromEdge | NavigationState::DirectFlight => {
                handover.final_target
            }
            _ => continue,
        };

        let to_target = target - pos;
        let distance = to_target.length();

        if distance < nav_target.arrival_tolerance {
            // Close enough, stop flying
            intent.set_fly(0.0);
            intent.set_fly_horizontal(0.0);
            continue;
        }

        let dir = to_target / distance;

        // Apply flight intent based on direction
        intent.set_fly_horizontal(dir.x * nav_target.flight_speed);
        intent.set_fly(dir.y * nav_target.flight_speed);
        intent.set_fly_speed(nav_target.flight_speed);
    }
}

/// Clean up navigation components when navigation is complete and target removed.
fn cleanup_completed_navigation(
    mut commands: Commands,
    query: Query<(Entity, &NavigationHandover), Without<NavigationTarget>>,
) {
    for (entity, _handover) in &query {
        commands.entity(entity).remove::<NavigationHandover>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bounds_contains() {
        let bounds = NavigationBounds {
            min: Vec2::ZERO,
            max: Vec2::new(100.0, 100.0),
            edge_margin: 10.0,
        };

        assert!(bounds.contains(Vec2::new(50.0, 50.0)));
        assert!(bounds.contains(Vec2::new(0.0, 0.0)));
        assert!(bounds.contains(Vec2::new(100.0, 100.0)));
        assert!(!bounds.contains(Vec2::new(-1.0, 50.0)));
        assert!(!bounds.contains(Vec2::new(101.0, 50.0)));
    }

    #[test]
    fn test_bounds_contains_with_margin() {
        let bounds = NavigationBounds {
            min: Vec2::ZERO,
            max: Vec2::new(100.0, 100.0),
            edge_margin: 10.0,
        };

        assert!(bounds.contains_with_margin(Vec2::new(50.0, 50.0)));
        assert!(bounds.contains_with_margin(Vec2::new(10.0, 10.0)));
        assert!(bounds.contains_with_margin(Vec2::new(90.0, 90.0)));
        assert!(!bounds.contains_with_margin(Vec2::new(5.0, 50.0))); // Too close to edge
        assert!(!bounds.contains_with_margin(Vec2::new(95.0, 50.0))); // Too close to edge
    }

    #[test]
    fn test_scenario_detection() {
        let bounds = NavigationBounds {
            min: Vec2::ZERO,
            max: Vec2::new(100.0, 100.0),
            edge_margin: 10.0,
        };

        // Both inside
        assert_eq!(
            NavigationScenario::from_positions(Vec2::new(50.0, 50.0), Vec2::new(60.0, 60.0), &bounds),
            NavigationScenario::BothInside
        );

        // Start inside, target outside
        assert_eq!(
            NavigationScenario::from_positions(Vec2::new(50.0, 50.0), Vec2::new(150.0, 50.0), &bounds),
            NavigationScenario::StartInsideTargetOutside
        );

        // Start outside, target inside
        assert_eq!(
            NavigationScenario::from_positions(Vec2::new(-50.0, 50.0), Vec2::new(50.0, 50.0), &bounds),
            NavigationScenario::StartOutsideTargetInside
        );

        // Both outside
        assert_eq!(
            NavigationScenario::from_positions(Vec2::new(-50.0, 50.0), Vec2::new(150.0, 50.0), &bounds),
            NavigationScenario::BothOutside
        );
    }

    #[test]
    fn test_closest_edge_point() {
        let bounds = NavigationBounds {
            min: Vec2::ZERO,
            max: Vec2::new(100.0, 100.0),
            edge_margin: 10.0,
        };

        // From outside left, going right
        let edge = bounds.closest_edge_point(Vec2::new(-50.0, 50.0), Vec2::new(50.0, 50.0));
        assert!((edge.x - 10.0).abs() < 0.1); // Should hit left margin
        assert!((edge.y - 50.0).abs() < 0.1);

        // From inside, going outside right
        let edge = bounds.closest_edge_point(Vec2::new(50.0, 50.0), Vec2::new(150.0, 50.0));
        assert!((edge.x - 90.0).abs() < 0.1); // Should hit right margin
        assert!((edge.y - 50.0).abs() < 0.1);
    }

    #[test]
    fn test_navigation_handover_state() {
        let mut handover = NavigationHandover::new(Vec2::new(100.0, 100.0));

        assert!(!handover.is_flying());
        assert!(!handover.is_following_path());
        assert!(!handover.is_arrived());

        handover.state = NavigationState::FlyingToEdge;
        assert!(handover.is_flying());

        handover.state = NavigationState::FollowingPath;
        assert!(handover.is_following_path());

        handover.state = NavigationState::Arrived;
        assert!(handover.is_arrived());
    }
}
