//! Navigation handover utilities for bevy_northstar integration.
//!
//! This module provides types and utilities for handling transitions between
//! grid-based pathfinding (bevy_northstar) and free flight when actors need
//! to navigate to/from positions outside the navigation grid.
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
//! This module provides utilities to handle all permutations:
//!
//! 1. **Both in grid** → Normal pathfinding (use `clamp_to_grid` for safety)
//! 2. **Start in, target out** → Navigate to edge, then fly to target
//! 3. **Start out, target in** → Fly to edge, then navigate to target
//! 4. **Both out** → Direct flight
//!
//! # Usage
//!
//! ```rust,ignore
//! use msg_character_controller::navigation::*;
//!
//! // Add NavigationBounds component to your grid entity
//! commands.spawn((
//!     Grid::new(...),
//!     NavigationBounds::from_grid(192, 192, TILE_SIZE),
//! ));
//!
//! // In your navigation system, clamp targets to grid bounds
//! fn set_target(
//!     mut actors: Query<&mut TargetPos>,
//!     grids: Query<&NavigationBounds>,
//! ) {
//!     let bounds = grids.single();
//!     for mut target in &mut actors {
//!         // Clamp target to grid - northstar will always succeed
//!         target.0 = bounds.clamp_to_grid(target.0);
//!     }
//! }
//!
//! // Or use NavigationHandover for full state machine control
//! fn navigate(
//!     actors: Query<(&Transform, &mut NavigationHandover)>,
//!     grids: Query<&NavigationBounds>,
//! ) {
//!     let bounds = grids.single();
//!     for (transform, mut handover) in &mut actors {
//!         let scenario = handover.evaluate(transform.translation.truncate(), bounds);
//!         match scenario {
//!             NavigationScenario::BothInside => { /* use northstar */ }
//!             NavigationScenario::StartOutsideTargetInside => { /* fly to edge first */ }
//!             // ...
//!         }
//!     }
//! }
//! ```

use bevy::prelude::*;

/// Component defining the navigable grid bounds.
///
/// Add this to your bevy_northstar Grid entity. Supports multiple grids
/// with different bounds in the same world.
#[derive(Component, Clone, Debug, Reflect)]
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

    /// Clamp a position to be inside the grid bounds.
    ///
    /// Use this to ensure TargetPos is always valid for northstar.
    /// Positions outside the grid will be moved to the nearest edge.
    pub fn clamp_to_grid(&self, pos: Vec2) -> Vec2 {
        Vec2::new(
            pos.x.clamp(self.min.x, self.max.x),
            pos.y.clamp(self.min.y, self.max.y),
        )
    }

    /// Clamp a position to be inside the grid bounds with margin.
    ///
    /// More conservative than `clamp_to_grid` - keeps positions away from edges.
    pub fn clamp_to_grid_with_margin(&self, pos: Vec2) -> Vec2 {
        Vec2::new(
            pos.x.clamp(self.min.x + self.edge_margin, self.max.x - self.edge_margin),
            pos.y.clamp(self.min.y + self.edge_margin, self.max.y - self.edge_margin),
        )
    }

    /// Find the closest point on the grid boundary (with margin applied).
    ///
    /// This is used to find where an actor should enter/exit the grid
    /// when transitioning between free flight and pathfinding.
    pub fn closest_edge_point(&self, from: Vec2, toward: Vec2) -> Vec2 {
        let inner_min = self.min + Vec2::splat(self.edge_margin);
        let inner_max = self.max - Vec2::splat(self.edge_margin);

        let dir = (toward - from).normalize_or_zero();

        if dir == Vec2::ZERO {
            return Vec2::new(
                from.x.clamp(inner_min.x, inner_max.x),
                from.y.clamp(inner_min.y, inner_max.y),
            );
        }

        let mut best_point = None;
        let mut best_dist = f32::INFINITY;

        let edges = [
            (inner_min.x, true),  // left edge
            (inner_max.x, true),  // right edge
            (inner_min.y, false), // bottom edge
            (inner_max.y, false), // top edge
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
                continue;
            }

            let point = from + dir * t;

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

        Vec2::new(
            pos.x.clamp(inner_min.x, inner_max.x),
            pos.y.clamp(inner_min.y, inner_max.y),
        )
    }
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

    /// Check if pathfinding can be used (at least one position in grid).
    pub fn can_use_pathfinding(&self) -> bool {
        !matches!(self, Self::BothOutside)
    }

    /// Check if free flight is needed (at least one position outside grid).
    pub fn needs_free_flight(&self) -> bool {
        !matches!(self, Self::BothInside)
    }
}

/// Navigation state machine for handling grid/free-flight transitions.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
pub enum NavigationState {
    /// No active navigation target.
    #[default]
    Idle,

    /// Following a bevy_northstar path.
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

/// Component tracking navigation handover state.
///
/// Add this to actors that need to navigate across grid boundaries.
#[derive(Component, Clone, Debug, Default, Reflect)]
pub struct NavigationHandover {
    /// Current state of the navigation state machine.
    pub state: NavigationState,
    /// The intermediate edge point (if transitioning at grid boundary).
    pub edge_point: Option<Vec2>,
    /// The final target position (may be outside grid).
    pub final_target: Vec2,
    /// The scenario determined when navigation started.
    pub scenario: NavigationScenario,
}

impl NavigationHandover {
    /// Create a new handover for navigating to a target.
    pub fn new(final_target: Vec2) -> Self {
        Self {
            state: NavigationState::Idle,
            edge_point: None,
            final_target,
            scenario: NavigationScenario::BothInside,
        }
    }

    /// Evaluate the navigation scenario and set up the state machine.
    ///
    /// Returns the scenario and updates internal state. Call this when
    /// setting a new navigation target.
    pub fn evaluate(&mut self, start: Vec2, target: Vec2, bounds: &NavigationBounds) -> NavigationScenario {
        self.final_target = target;
        self.scenario = NavigationScenario::from_positions(start, target, bounds);

        match self.scenario {
            NavigationScenario::BothInside => {
                self.state = NavigationState::FollowingPath;
                self.edge_point = None;
            }
            NavigationScenario::StartInsideTargetOutside => {
                // Navigate to edge first, then fly
                let edge = bounds.closest_edge_point(start, target);
                self.edge_point = Some(edge);
                self.state = NavigationState::FollowingPath;
            }
            NavigationScenario::StartOutsideTargetInside => {
                // Fly to edge first, then pathfind
                let edge = bounds.closest_edge_point(start, target);
                self.edge_point = Some(edge);
                self.state = NavigationState::FlyingToEdge;
            }
            NavigationScenario::BothOutside => {
                self.edge_point = None;
                self.state = NavigationState::DirectFlight;
            }
        }

        self.scenario
    }

    /// Get the current navigation target (may be edge point or final target).
    ///
    /// Use this to get the appropriate target for either pathfinding or flight.
    pub fn current_target(&self) -> Vec2 {
        match self.state {
            NavigationState::FollowingPath => {
                // If we have an edge point, navigate to edge first
                self.edge_point.unwrap_or(self.final_target)
            }
            NavigationState::FlyingToEdge => {
                self.edge_point.unwrap_or(self.final_target)
            }
            NavigationState::FlyingFromEdge | NavigationState::DirectFlight => {
                self.final_target
            }
            NavigationState::Idle | NavigationState::Arrived => {
                self.final_target
            }
        }
    }

    /// Notify that the current waypoint was reached.
    ///
    /// Call this when the actor reaches the edge point or final target.
    /// Returns true if navigation is complete.
    pub fn waypoint_reached(&mut self) -> bool {
        match self.state {
            NavigationState::FollowingPath if self.edge_point.is_some() => {
                // Reached edge, now fly to final target
                self.state = NavigationState::FlyingFromEdge;
                false
            }
            NavigationState::FollowingPath => {
                // Reached final target via pathfinding
                self.state = NavigationState::Arrived;
                true
            }
            NavigationState::FlyingToEdge => {
                // Reached edge, now pathfind to target
                self.state = NavigationState::FollowingPath;
                self.edge_point = None; // Clear edge point, now targeting final
                false
            }
            NavigationState::FlyingFromEdge | NavigationState::DirectFlight => {
                self.state = NavigationState::Arrived;
                true
            }
            _ => true,
        }
    }

    /// Reset to idle state.
    pub fn reset(&mut self) {
        self.state = NavigationState::Idle;
        self.edge_point = None;
        self.scenario = NavigationScenario::BothInside;
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

    /// Check if currently following a path (use northstar).
    pub fn is_following_path(&self) -> bool {
        self.state == NavigationState::FollowingPath
    }

    /// Check if idle (no active navigation).
    pub fn is_idle(&self) -> bool {
        self.state == NavigationState::Idle
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
        assert!(!bounds.contains_with_margin(Vec2::new(5.0, 50.0)));
        assert!(!bounds.contains_with_margin(Vec2::new(95.0, 50.0)));
    }

    #[test]
    fn test_clamp_to_grid() {
        let bounds = NavigationBounds {
            min: Vec2::ZERO,
            max: Vec2::new(100.0, 100.0),
            edge_margin: 10.0,
        };

        // Inside - unchanged
        assert_eq!(bounds.clamp_to_grid(Vec2::new(50.0, 50.0)), Vec2::new(50.0, 50.0));

        // Outside - clamped
        assert_eq!(bounds.clamp_to_grid(Vec2::new(-50.0, 50.0)), Vec2::new(0.0, 50.0));
        assert_eq!(bounds.clamp_to_grid(Vec2::new(150.0, 50.0)), Vec2::new(100.0, 50.0));
        assert_eq!(bounds.clamp_to_grid(Vec2::new(50.0, -50.0)), Vec2::new(50.0, 0.0));
        assert_eq!(bounds.clamp_to_grid(Vec2::new(50.0, 150.0)), Vec2::new(50.0, 100.0));
    }

    #[test]
    fn test_scenario_detection() {
        let bounds = NavigationBounds {
            min: Vec2::ZERO,
            max: Vec2::new(100.0, 100.0),
            edge_margin: 10.0,
        };

        assert_eq!(
            NavigationScenario::from_positions(Vec2::new(50.0, 50.0), Vec2::new(60.0, 60.0), &bounds),
            NavigationScenario::BothInside
        );

        assert_eq!(
            NavigationScenario::from_positions(Vec2::new(50.0, 50.0), Vec2::new(150.0, 50.0), &bounds),
            NavigationScenario::StartInsideTargetOutside
        );

        assert_eq!(
            NavigationScenario::from_positions(Vec2::new(-50.0, 50.0), Vec2::new(50.0, 50.0), &bounds),
            NavigationScenario::StartOutsideTargetInside
        );

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
        assert!((edge.x - 10.0).abs() < 0.1);
        assert!((edge.y - 50.0).abs() < 0.1);

        // From inside, going outside right
        let edge = bounds.closest_edge_point(Vec2::new(50.0, 50.0), Vec2::new(150.0, 50.0));
        assert!((edge.x - 90.0).abs() < 0.1);
        assert!((edge.y - 50.0).abs() < 0.1);
    }

    #[test]
    fn test_handover_evaluate_both_inside() {
        let bounds = NavigationBounds::from_grid(100, 100, 1.0);
        let mut handover = NavigationHandover::new(Vec2::ZERO);

        let scenario = handover.evaluate(Vec2::new(10.0, 10.0), Vec2::new(50.0, 50.0), &bounds);

        assert_eq!(scenario, NavigationScenario::BothInside);
        assert!(handover.is_following_path());
        assert_eq!(handover.current_target(), Vec2::new(50.0, 50.0));
    }

    #[test]
    fn test_handover_evaluate_start_outside() {
        let bounds = NavigationBounds::from_grid(100, 100, 1.0);
        let mut handover = NavigationHandover::new(Vec2::ZERO);

        let scenario = handover.evaluate(Vec2::new(-50.0, 50.0), Vec2::new(50.0, 50.0), &bounds);

        assert_eq!(scenario, NavigationScenario::StartOutsideTargetInside);
        assert!(handover.is_flying());
        assert!(handover.edge_point.is_some());
    }

    #[test]
    fn test_handover_waypoint_transitions() {
        let bounds = NavigationBounds::from_grid(100, 100, 1.0);
        let mut handover = NavigationHandover::new(Vec2::ZERO);

        // Start outside, target inside
        handover.evaluate(Vec2::new(-50.0, 50.0), Vec2::new(50.0, 50.0), &bounds);
        assert!(handover.is_flying());

        // Reach edge
        let complete = handover.waypoint_reached();
        assert!(!complete);
        assert!(handover.is_following_path());

        // Reach final target
        let complete = handover.waypoint_reached();
        assert!(complete);
        assert!(handover.is_arrived());
    }

    #[test]
    fn test_navigation_handover_state() {
        let mut handover = NavigationHandover::new(Vec2::new(100.0, 100.0));

        assert!(!handover.is_flying());
        assert!(!handover.is_following_path());
        assert!(!handover.is_arrived());
        assert!(handover.is_idle());

        handover.state = NavigationState::FlyingToEdge;
        assert!(handover.is_flying());

        handover.state = NavigationState::FollowingPath;
        assert!(handover.is_following_path());

        handover.state = NavigationState::Arrived;
        assert!(handover.is_arrived());
    }
}
