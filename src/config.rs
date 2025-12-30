//! Controller configuration components.
//!
//! This module defines the core configuration for character controllers,
//! including float height, spring parameters, slope limits, and stair stepping.

use bevy::prelude::*;
use std::time::Duration;

#[cfg(feature = "rapier2d")]
use bevy_rapier2d::prelude::{ExternalForce, ExternalImpulse, ReadMassProperties};

use crate::{collision::CollisionData, intent::MovementIntent};

/// Type of jump based on the surface being jumped from.
///
/// This determines the direction of the jump impulse:
/// - `Ground`: Jump straight up (along ground normal)
/// - `LeftWall`: Jump diagonally up-right (away from left wall)
/// - `RightWall`: Jump diagonally up-left (away from right wall)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Reflect)]
pub enum JumpType {
    /// Normal ground jump - impulse along ground normal or ideal up.
    #[default]
    Ground,
    /// Wall jump from left wall - impulse diagonally up-right.
    LeftWall,
    /// Wall jump from right wall - impulse diagonally up-left.
    RightWall,
}

/// Core character controller component.
///
/// This is the **central hub** for all character controller state.
/// It contains RESULT states - not raw measurements that need computation.
///
/// # Contact States
///
/// Each direction stores `Option<CollisionData>` with full collision information.
/// - `floor`: Ground collision data (when detected within raycast range)
/// - `ceiling`: Ceiling collision data (when detected)
/// - `left_wall`: Left wall collision data (when detected)
/// - `right_wall`: Right wall collision data (when detected)
///
/// # Force Isolation
///
/// The controller tracks forces it applies internally to avoid interfering with
/// external user forces on ExternalForce/ExternalTorque components. At the start
/// of each frame, previously applied forces are subtracted, and at the end of
/// the frame, accumulated forces are applied. This ensures the controller's
/// forces are "isolated" and don't accumulate across frames.
#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
#[require(MovementIntent)]
#[cfg(feature = "rapier2d")]
#[require(ReadMassProperties, ExternalForce, ExternalImpulse)]
pub struct CharacterController {
    // === Collision Data (Option<CollisionData> for each direction) ===
    /// Floor collision data. Contains distance, normal, point, and entity.
    /// None if no floor detected within raycast range.
    #[reflect(ignore)]
    pub floor: Option<CollisionData>,
    /// Ceiling collision data. Contains distance, normal, point, and entity.
    /// None if no ceiling detected within range.
    #[reflect(ignore)]
    pub ceiling: Option<CollisionData>,
    /// Left wall collision data. Contains distance, normal, point, and entity.
    /// None if no left wall detected within range.
    #[reflect(ignore)]
    pub left_wall: Option<CollisionData>,
    /// Right wall collision data. Contains distance, normal, point, and entity.
    /// None if no right wall detected within range.
    #[reflect(ignore)]
    pub right_wall: Option<CollisionData>,

    // === Derived Ground State ===
    /// Slope angle in radians (0 = flat). Valid when floor is Some.
    pub slope_angle: f32,
    /// Whether a valid step was detected (for stair stepping).
    pub step_detected: bool,
    /// Height of the detected step (if any).
    pub step_height: f32,
    /// Timer tracking time since last grounded (for coyote time).
    /// When grounded, this timer is reset. When not grounded, it ticks.
    /// Coyote time is valid while the timer has not finished.
    #[reflect(ignore)]
    pub coyote_timer: Timer,
    /// The type of jump that should be performed based on last contact.
    /// Updated by wall contact detection system. Used by apply_jump to determine
    /// jump direction. Ground contact takes priority over wall contact.
    pub last_jump_type: JumpType,

    // === Stair Climbing State ===
    /// Active stair height when climbing. This is added to the riding height
    /// temporarily to raise the character over the step. Resets to 0 when
    /// no stair is detected.
    pub active_stair_height: f32,
    /// Timer for filtering spring forces after upward propulsion (jump or fly up).
    /// When upward propulsion occurs, this timer is reset. Spring force filtering
    /// is active while the timer has not finished.
    #[reflect(ignore)]
    pub jump_spring_filter_timer: Timer,

    /// Timer tracking time since last jump for early jump cancellation.
    /// When a jump occurs, this timer is reset. Used by the fall gravity system
    /// to determine if the player can cancel the jump early.
    #[reflect(ignore)]
    pub jumped_timer: Timer,

    /// Timer for applying fall gravity after jump cancellation.
    /// When fall gravity is triggered, this timer is reset and fall
    /// gravity is applied while the timer has not finished.
    #[reflect(ignore)]
    pub fall_gravity_timer: Timer,

    /// Timer for blocking movement toward the wall after a wall jump.
    /// When a wall jump occurs, this timer is reset. While active, movement
    /// toward the wall (opposing the jump direction) is blocked.
    #[reflect(ignore)]
    pub wall_jump_movement_block_timer: Timer,

    /// The direction that is blocked during wall jump movement blocking.
    /// Positive = block rightward movement (jumped from right wall)
    /// Negative = block leftward movement (jumped from left wall)
    /// Zero = no blocking
    pub wall_jump_blocked_direction: f32,

    // === Intent State (set by evaluate_intent, used by force systems) ===
    /// Whether the character intends to propel upward this frame.
    /// Set by evaluate_intent system based on MovementIntent.
    /// Used by spring system to filter downward forces immediately.
    pub intends_upward_propulsion: bool,

    // === Gravity ===
    /// Gravity vector affecting this character.
    /// Used for floating spring, fall gravity, and jump countering.
    pub gravity: Vec2,

    // === Force Accumulation (internal) ===
    /// Forces accumulated during the current frame (not yet applied to ExternalForce).
    #[reflect(ignore)]
    pub(crate) accumulated_force: Vec2,
    /// Torque accumulated during the current frame (not yet applied to ExternalForce).
    #[reflect(ignore)]
    pub(crate) accumulated_torque: f32,
    /// Forces that were applied to ExternalForce last frame (to be subtracted next frame).
    #[reflect(ignore)]
    pub(crate) applied_force: Vec2,
    /// Torque that was applied to ExternalForce last frame (to be subtracted next frame).
    #[reflect(ignore)]
    pub(crate) applied_torque: f32,

    // === Internal (used by systems, kept pub(crate)) ===
    /// Distance from collider center to bottom (auto-detected from Collider).
    /// For a capsule, this is half_height + radius.
    pub(crate) collider_bottom_offset: f32,

    // === Stair Configuration ===
    /// Configuration for stair stepping behavior.
    /// Set to `Some(StairConfig::default())` by default (enabled).
    /// Set to `None` to disable stair stepping entirely.
    pub stair_config: Option<StairConfig>,
}

impl Default for CharacterController {
    fn default() -> Self {
        Self {
            // Collision data (None = not detected)
            floor: None,
            ceiling: None,
            left_wall: None,
            right_wall: None,
            // Derived ground state
            slope_angle: 0.0,
            step_detected: false,
            step_height: 0.0,
            // Coyote timer starts finished (not grounded) - will be reset when grounded
            // Duration is set by the system based on config.coyote_time
            coyote_timer: Timer::new(Duration::ZERO, TimerMode::Once),
            // Default to ground jump type
            last_jump_type: JumpType::Ground,
            // Stair climbing state
            active_stair_height: 0.0,
            // Jump spring filter timer starts finished (no recent propulsion)
            // Duration is set by the system based on config.jump_spring_filter_duration
            jump_spring_filter_timer: Timer::new(Duration::ZERO, TimerMode::Once),
            // Jumped timer starts finished (no recent jump)
            // Duration is set by the system based on config.jump_cancel_window
            jumped_timer: Timer::new(Duration::ZERO, TimerMode::Once),
            // Fall gravity timer starts finished (no active fall gravity)
            // Duration is set by the system based on config.fall_gravity_duration
            fall_gravity_timer: Timer::new(Duration::ZERO, TimerMode::Once),
            // Wall jump movement block timer starts finished (no active blocking)
            // Duration is set by the system based on config.wall_jump_movement_block_duration
            wall_jump_movement_block_timer: Timer::new(Duration::ZERO, TimerMode::Once),
            // No direction blocked initially
            wall_jump_blocked_direction: 0.0,
            // Intent state
            intends_upward_propulsion: false,
            // Gravity
            gravity: Vec2::new(0.0, -980.0),
            // Force accumulation (internal)
            accumulated_force: Vec2::ZERO,
            accumulated_torque: 0.0,
            applied_force: Vec2::ZERO,
            applied_torque: 0.0,
            // Internal
            collider_bottom_offset: 0.0,
            // Stair configuration (enabled by default)
            stair_config: Some(StairConfig::default()),
        }
    }
}

impl CharacterController {
    /// Create a new controller with default gravity.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a new controller with custom gravity.
    pub fn with_gravity(gravity: Vec2) -> Self {
        Self {
            gravity,
            ..default()
        }
    }

    /// Set the gravity vector.
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.gravity = gravity;
    }

    /// Get the ideal "up" direction for raycasts, derived from gravity.
    ///
    /// This returns the opposite of the normalized gravity vector.
    /// Raycasts use this to ensure they work correctly regardless of
    /// the actor's physical rotation.
    ///
    /// If gravity is zero, defaults to `Vec2::Y`.
    #[inline]
    pub fn ideal_up(&self) -> Vec2 {
        let normalized = self.gravity.normalize_or_zero();
        if normalized == Vec2::ZERO {
            Vec2::Y
        } else {
            -normalized
        }
    }

    /// Get the ideal "down" direction for raycasts (same as gravity direction).
    #[inline]
    pub fn ideal_down(&self) -> Vec2 {
        -self.ideal_up()
    }

    /// Get the ideal "right" direction for raycasts (perpendicular to ideal up).
    #[inline]
    pub fn ideal_right(&self) -> Vec2 {
        let up = self.ideal_up();
        Vec2::new(up.y, -up.x)
    }

    /// Get the ideal "left" direction for raycasts (perpendicular to ideal up).
    #[inline]
    pub fn ideal_left(&self) -> Vec2 {
        -self.ideal_right()
    }

    /// Get the angle of the ideal up direction (radians from world +X axis).
    ///
    /// This is used for rotating shapes in raycasts to align with the ideal
    /// coordinate system.
    #[inline]
    pub fn ideal_up_angle(&self) -> f32 {
        self.ideal_up().to_angle()
    }

    /// Builder: set stair configuration.
    pub fn with_stair_config(mut self, config: StairConfig) -> Self {
        self.stair_config = Some(config);
        self
    }

    /// Builder: disable stair stepping.
    pub fn without_stair_stepping(mut self) -> Self {
        self.stair_config = None;
        self
    }

    /// Set stair configuration at runtime.
    pub fn set_stair_config(&mut self, config: Option<StairConfig>) {
        self.stair_config = config;
    }

    /// Enable or disable stair stepping at runtime.
    pub fn set_stair_stepping_enabled(&mut self, enabled: bool) {
        if enabled {
            if self.stair_config.is_none() {
                self.stair_config = Some(StairConfig::default());
            }
        } else {
            self.stair_config = None;
        }
    }

    /// Check if stair stepping is enabled.
    pub fn stair_stepping_enabled(&self) -> bool {
        self.stair_config.as_ref().is_some_and(|c| c.enabled)
    }

    /// Check if grounded (floor detected within float_height + ground_tolerance).
    pub fn is_grounded(&self, config: &ControllerConfig) -> bool {
        if let Some(ref floor) = self.floor {
            let riding_height = self.riding_height(config);
            floor.distance <= riding_height + config.ground_tolerance
        } else {
            false
        }
    }

    /// Get the ground normal if floor is detected.
    pub fn ground_normal(&self) -> Vec2 {
        self.floor.as_ref().map(|f| f.normal).unwrap_or(Vec2::Y)
    }

    /// Get the ground entity if floor is detected.
    pub fn ground_entity(&self) -> Option<Entity> {
        self.floor.as_ref().and_then(|f| f.entity)
    }

    /// Get the ground tangent vector (for movement direction along slopes).
    pub fn ground_tangent(&self) -> Vec2 {
        let normal = self.ground_normal();
        Vec2::new(normal.y, -normal.x)
    }

    /// Check if touching any wall.
    pub fn touching_wall(&self) -> bool {
        self.left_wall.is_some() || self.right_wall.is_some()
    }

    /// Check if touching left wall.
    pub fn touching_left_wall(&self) -> bool {
        self.left_wall.is_some()
    }

    /// Check if touching right wall.
    pub fn touching_right_wall(&self) -> bool {
        self.right_wall.is_some()
    }

    /// Check if touching ceiling.
    pub fn touching_ceiling(&self) -> bool {
        self.ceiling.is_some()
    }

    /// Get the wall normal if touching a wall in the given direction.
    pub fn wall_normal(&self, direction: f32) -> Option<Vec2> {
        if direction < 0.0 {
            self.left_wall.as_ref().map(|w| w.normal)
        } else if direction > 0.0 {
            self.right_wall.as_ref().map(|w| w.normal)
        } else {
            None
        }
    }

    /// Check if on a slope that requires extra handling.
    pub fn is_on_slope(&self, config: &ControllerConfig) -> bool {
        self.is_grounded(config) && self.slope_angle.abs() > 0.1
    }

    /// Get the raw distance to ground (for debugging/testing).
    pub fn ground_distance(&self) -> f32 {
        self.floor.as_ref().map(|f| f.distance).unwrap_or(f32::MAX)
    }

    /// Check if ground was detected by raycast (for debugging/testing).
    pub fn ground_detected(&self) -> bool {
        self.floor.is_some()
    }

    /// Get the ground contact point in world space (for debugging/testing).
    pub fn ground_contact_point(&self) -> Vec2 {
        self.floor.as_ref().map(|f| f.point).unwrap_or(Vec2::ZERO)
    }

    /// Reset all detection state (called at start of each frame).
    pub(crate) fn reset_detection_state(&mut self) {
        // Reset all collision data
        self.floor = None;
        self.ceiling = None;
        self.left_wall = None;
        self.right_wall = None;

        // Reset derived state
        self.slope_angle = 0.0;
        self.step_detected = false;
        self.step_height = 0.0;
        // Note: active_stair_height is NOT reset here - it's managed by the stair climbing system
    }

    /// Get the base riding height (desired height from ground to collider center).
    /// This is float_height + collider_bottom_offset (half_height + radius for capsule).
    /// Does NOT include active_stair_height - use effective_riding_height for that.
    #[inline]
    pub fn riding_height(&self, config: &ControllerConfig) -> f32 {
        config.float_height + self.collider_bottom_offset
    }

    /// Get the effective riding height including stair climbing adjustment.
    /// This is riding_height + active_stair_height.
    /// Use this for the floating spring system when climbing stairs.
    #[inline]
    pub fn effective_riding_height(&self, config: &ControllerConfig) -> f32 {
        self.riding_height(config) + self.active_stair_height
    }

    /// Get the capsule half height (half_height + radius for capsule).
    /// This is the collider_bottom_offset.
    #[inline]
    pub fn capsule_half_height(&self) -> f32 {
        self.collider_bottom_offset
    }

    /// Check if within spring active range.
    /// Active when:
    /// - Distance <= float_height + ground_tolerance (riding range)
    /// - Distance > capsule_half_height - EPSILON (physics collision threshold)
    pub fn in_spring_range(&self, config: &ControllerConfig) -> bool {
        if let Some(ref floor) = self.floor {
            let riding_height = self.riding_height(config);
            let max_range = riding_height + config.ground_tolerance;
            let min_range = self.collider_bottom_offset - f32::EPSILON;
            floor.distance <= max_range && floor.distance > min_range
        } else {
            false
        }
    }

    /// Check if physics collision should take over (very close to ground).
    /// True when floor distance < capsule_half_height - EPSILON.
    pub fn physics_collision_active(&self) -> bool {
        if let Some(ref floor) = self.floor {
            floor.distance < self.collider_bottom_offset - f32::EPSILON
        } else {
            false
        }
    }

    /// Record an upward propulsion event (jump or fly up).
    /// This enables temporary filtering of downward spring forces.
    pub fn record_upward_propulsion(&mut self, filter_duration: f32) {
        if filter_duration > 0.0 {
            self.jump_spring_filter_timer
                .set_duration(Duration::from_secs_f32(filter_duration));
            self.jump_spring_filter_timer.reset();
        }
    }

    /// Check if within the jump spring filter window.
    /// Returns true if downward spring forces should be filtered.
    pub fn in_jump_spring_filter_window(&self) -> bool {
        !self.jump_spring_filter_timer.finished()
    }

    /// Check if the character has upward intent (jumping or flying up).
    /// Returns true if downward forces (spring, gravity) should be filtered.
    ///
    /// This combines same-frame intent (`intends_upward_propulsion`) with
    /// cross-frame filtering (`jump_spring_filter_window`) for consistent behavior.
    #[inline]
    pub fn upward_intent(&self) -> bool {
        self.intends_upward_propulsion || self.in_jump_spring_filter_window()
    }

    /// Tick the coyote timer (call when not grounded).
    pub fn tick_coyote_timer(&mut self, delta: Duration) {
        self.coyote_timer.tick(delta);
    }

    /// Reset the coyote timer (call when grounded).
    pub fn reset_coyote_timer(&mut self, coyote_time: f32) {
        if coyote_time > 0.0 {
            self.coyote_timer
                .set_duration(Duration::from_secs_f32(coyote_time));
            self.coyote_timer.reset();
        }
    }

    /// Check if within coyote time window.
    /// Returns true if the character can still jump after leaving ground or wall.
    pub fn in_coyote_time(&self) -> bool {
        !self.coyote_timer.finished()
    }

    // === Fall Gravity Timer Methods ===

    /// Record a jump event for fall gravity tracking.
    /// This starts the jumped timer which tracks how long since the last jump.
    pub fn record_jump(&mut self, jump_cancel_window: f32) {
        if jump_cancel_window > 0.0 {
            self.jumped_timer
                .set_duration(Duration::from_secs_f32(jump_cancel_window));
            self.jumped_timer.reset();
        }
    }

    /// Check if we are within the jump cancel window.
    /// Returns true if the jump was recent enough to be cancelled.
    pub fn in_jump_cancel_window(&self) -> bool {
        !self.jumped_timer.finished()
    }

    /// Trigger fall gravity for the specified duration.
    /// This starts the fall gravity timer.
    pub fn trigger_fall_gravity(&mut self, fall_gravity_duration: f32) {
        if fall_gravity_duration > 0.0 {
            self.fall_gravity_timer
                .set_duration(Duration::from_secs_f32(fall_gravity_duration));
            self.fall_gravity_timer.reset();
        }
    }

    /// Check if fall gravity is currently active.
    /// Returns true if fall gravity should be applied.
    pub fn fall_gravity_active(&self) -> bool {
        !self.fall_gravity_timer.finished()
    }

    // === Wall Jump Movement Block Methods ===

    /// Record a wall jump movement block event.
    /// This starts the wall jump movement block timer and stores the blocked direction.
    ///
    /// # Arguments
    /// * `duration` - How long to block movement (in seconds)
    /// * `blocked_direction` - The direction to block: positive = rightward, negative = leftward
    pub fn record_wall_jump_movement_block(&mut self, duration: f32, blocked_direction: f32) {
        if duration > 0.0 {
            self.wall_jump_movement_block_timer
                .set_duration(Duration::from_secs_f32(duration));
            self.wall_jump_movement_block_timer.reset();
            self.wall_jump_blocked_direction = blocked_direction;
        }
    }

    /// Check if wall jump movement blocking is currently active.
    /// Returns true if movement toward the wall should be blocked.
    pub fn wall_jump_movement_blocked(&self) -> bool {
        !self.wall_jump_movement_block_timer.finished()
    }

    /// Get the direction that is currently blocked by wall jump movement blocking.
    /// Returns positive for rightward, negative for leftward, or 0 if not blocked.
    pub fn get_wall_jump_blocked_direction(&self) -> f32 {
        if self.wall_jump_movement_blocked() {
            self.wall_jump_blocked_direction
        } else {
            0.0
        }
    }

    // === Force Accumulation Methods ===

    /// Add force to the internal accumulator (called by controller systems).
    #[inline]
    pub(crate) fn add_force(&mut self, force: Vec2) {
        self.accumulated_force += force;
    }

    /// Add torque to the internal accumulator (called by controller systems).
    #[inline]
    pub(crate) fn add_torque(&mut self, torque: f32) {
        self.accumulated_torque += torque;
    }

    /// Reset intent state for a new frame.
    /// Called at the start of each frame before intent evaluation.
    pub(crate) fn reset_intent_state(&mut self) {
        self.intends_upward_propulsion = false;
    }

    /// Prepare for a new frame: returns the forces to subtract from ExternalForce.
    /// After calling this, accumulators are cleared and applied values are zeroed.
    pub(crate) fn prepare_new_frame(&mut self) -> (Vec2, f32) {
        let to_subtract = (self.applied_force, self.applied_torque);
        self.accumulated_force = Vec2::ZERO;
        self.accumulated_torque = 0.0;
        self.applied_force = Vec2::ZERO;
        self.applied_torque = 0.0;
        to_subtract
    }

    /// Finalize the frame: returns the accumulated forces to apply to ExternalForce.
    /// Moves accumulated values to applied for next frame's subtraction.
    pub(crate) fn finalize_frame(&mut self) -> (Vec2, f32) {
        let to_apply = (self.accumulated_force, self.accumulated_torque);
        self.applied_force = self.accumulated_force;
        self.applied_torque = self.accumulated_torque;
        to_apply
    }
}

/// Configuration parameters for the character controller.
///
/// All raycast lengths are DERIVED from float_height and other settings.
/// No hardcoded magic numbers.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct ControllerConfig {
    // === Float Settings ===
    /// Target height to float above ground (in world units/pixels).
    /// This is the distance from the BOTTOM of the collider to the ground.
    ///
    /// For a capsule with `Collider::capsule_y(half_height, radius)`, the system
    /// automatically detects the collider dimensions and adds them internally.
    /// So if you set float_height=1.0, the bottom of the capsule will float
    /// 1 pixel above the ground.
    pub float_height: f32,

    /// Tolerance below float_height where the spring is still active.
    /// The spring will restore riding_height when within this range.
    /// Total grounded range is: riding_height + ground_tolerance.
    pub ground_tolerance: f32,

    /// Distance for wall and ceiling detection (extends from collider surface).
    /// Also used as a buffer zone above riding_height where grounding_strength applies.
    pub grounding_distance: f32,

    /// Multiplier for downward spring force when character is within grounding_distance
    /// above the riding_height. This helps keep the character grounded when slightly floating.
    /// A value of 1.0 means no extra force, 2.0 doubles the downward force, etc.
    pub grounding_strength: f32,

    // === Spring Settings ===
    /// Spring strength for the floating system.
    pub spring_strength: f32,

    /// Spring damping coefficient.
    pub spring_damping: f32,

    /// Maximum spring force to apply.
    /// If None, uses the default formula-based clamp.
    pub spring_max_force: Option<f32>,

    /// Maximum vertical velocity at which spring force is still applied.
    /// If already moving upward (toward target) at or above this speed, no additional force is applied.
    /// This prevents overshooting by limiting acceleration when already moving fast enough.
    pub spring_max_velocity: Option<f32>,

    /// Duration (seconds) after jump/upward propulsion during which downward spring forces are filtered.
    /// During this window, only upward spring forces are applied to avoid counteracting the jump.
    /// Set to 0.0 to disable this filtering.
    pub jump_spring_filter_duration: f32,

    // === Movement Settings ===
    /// Maximum horizontal movement speed (units/second).
    pub max_speed: f32,

    /// Horizontal acceleration rate (units/second^2).
    pub acceleration: f32,

    /// Friction/deceleration when no input (0.0-1.0).
    pub friction: f32,

    /// Air control multiplier (0.0-1.0).
    pub air_control: f32,

    /// Whether the character can cling to walls by walking into them.
    /// When true (default), walking into a wall applies movement impulse normally.
    /// When false, movement intent toward a detected wall is rejected.
    pub wall_clinging: bool,

    /// Friction applied when clinging to a wall (0.0-1.0).
    /// Higher values make the character stick more firmly to the wall.
    pub wall_clinging_friction: f32,

    // === Slope Settings ===
    /// Maximum slope angle the character can walk up (radians).
    pub max_slope_angle: f32,

    /// Extra downward force when walking uphill.
    pub uphill_gravity_multiplier: f32,

    // === Sensor Settings (multipliers - actual lengths derived from float_height) ===
    /// Ground cast length = float_height * ground_cast_multiplier.
    /// Higher values detect ground from further away (good for falling).
    pub ground_cast_multiplier: f32,

    /// Width of the ground detection shapecast.
    pub ground_cast_width: f32,

    /// Wall cast length = ground_cast_width * wall_cast_multiplier.
    pub wall_cast_multiplier: f32,

    /// Height of wall detection shapecasts.
    pub wall_cast_height: f32,

    /// Ceiling cast length = float_height * ceiling_cast_multiplier.
    pub ceiling_cast_multiplier: f32,

    /// Width of ceiling detection shapecast.
    pub ceiling_cast_width: f32,

    // === Jump Settings ===
    /// Jump impulse strength (applied as velocity).
    pub jump_speed: f32,

    /// Coyote time duration in seconds.
    pub coyote_time: f32,

    /// Jump buffer duration in seconds.
    pub jump_buffer_time: f32,

    /// Gravity multiplier when jump is cancelled early.
    /// This multiplier is applied to gravity during the fall_gravity_duration
    /// window after the player releases the jump button or crosses the zenith.
    pub fall_gravity: f32,

    // === Wall Jump Settings ===
    /// Whether wall jumping is enabled.
    /// When enabled, the character can jump off walls when only touching a wall.
    pub wall_jumping: bool,

    /// Angle of wall jump from vertical (in radians).
    /// 0 = straight up, PI/4 (45°) = diagonal.
    /// The jump direction is angled away from the wall.
    pub wall_jump_angle: f32,

    /// Duration (seconds) after a wall jump during which movement toward the wall
    /// is blocked. This helps the player jump away from the wall correctly by
    /// preventing immediate movement back toward it.
    /// Default is 0.15 seconds (150ms).
    pub wall_jump_movement_block_duration: f32,

    /// How much downward velocity should be compensated on wall jumps (0.0-1.0).
    /// 0.0 = no compensation (wall jump adds to existing velocity)
    /// 1.0 = full compensation (wall jump cancels all downward velocity first)
    pub wall_jump_velocity_compensation: f32,

    /// Duration (seconds) after jumping during which the jump can be cancelled.
    /// If the player releases the jump button within this window OR crosses the
    /// zenith (starts moving downward), fall gravity will be triggered.
    pub jump_cancel_window: f32,

    /// Duration (seconds) for which fall gravity is applied after cancellation.
    /// During this time, gravity is multiplied by fall_gravity.
    pub fall_gravity_duration: f32,

    // === Upright Torque Settings ===
    /// Whether to apply torque to keep the character upright.
    pub upright_torque_enabled: bool,

    /// Strength of the upright torque spring.
    pub upright_torque_strength: f32,

    /// Damping coefficient for the upright torque.
    pub upright_torque_damping: f32,

    /// Target angle for upright torque (radians). None = derived from gravity via ideal_up_angle().
    pub upright_target_angle: Option<f32>,

    /// Maximum torque to apply for uprighting.
    /// If None, uses the default formula-based clamp.
    pub upright_max_torque: Option<f32>,

    /// Maximum angular velocity at which torque is still applied.
    /// If already rotating toward the target at or above this speed, no additional torque is applied.
    /// This prevents overshooting by limiting acceleration when already moving fast enough.
    pub upright_max_angular_velocity: Option<f32>,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            // Float settings
            float_height: 1.0,     // 1 pixel gap between collider bottom and ground
            ground_tolerance: 2.0, // tolerance for spring activation
            grounding_distance: 2.0, // distance for wall/ceiling detection and grounding buffer
            grounding_strength: 1.0, // multiplier for downward spring force (1.0 = no extra force)

            // Spring settings
            spring_strength: 300.0,
            spring_damping: 13.0,
            spring_max_force: None,
            spring_max_velocity: None,
            jump_spring_filter_duration: 0.15, // 150ms

            // Movement settings
            max_speed: 150.0,
            acceleration: 800.0,
            friction: 0.1,
            air_control: 0.3,
            wall_clinging: true, // Allow wall clinging by default
            wall_clinging_friction: 0.5, // Moderate wall friction by default

            // Slope settings
            max_slope_angle: std::f32::consts::FRAC_PI_3, // 60 degrees
            uphill_gravity_multiplier: 1.0,

            // Sensor settings (derived from float_height)
            ground_cast_multiplier: 1.0,
            ground_cast_width: 12.0,
            wall_cast_multiplier: 1.0,
            wall_cast_height: 12.0,
            ceiling_cast_multiplier: 1.0,
            ceiling_cast_width: 12.0,

            // Jump settings
            jump_speed: 120.0,
            coyote_time: 0.15,
            jump_buffer_time: 0.1,
            fall_gravity: 2.0,             // 2x gravity when jump is cancelled
            jump_cancel_window: 2.0,       // 2 seconds to cancel jump
            fall_gravity_duration: 0.3,    // 300ms of fall gravity

            // Wall jump settings
            wall_jumping: false,
            wall_jump_angle: std::f32::consts::FRAC_PI_4, // 45 degrees
            wall_jump_movement_block_duration: 0.15, // 150ms
            wall_jump_velocity_compensation: 0.5, // 50% downward velocity compensation

            // Upright torque settings
            upright_torque_enabled: true,
            upright_torque_strength: 120.0,
            upright_torque_damping: 5.0,
            upright_target_angle: None,
            upright_max_torque: None,
            upright_max_angular_velocity: Some(10.0),
        }
    }
}

impl ControllerConfig {
    /// Get the wall cast length (derived from ground_cast_width).
    #[inline]
    pub fn wall_cast_length(&self) -> f32 {
        self.ground_cast_width * self.wall_cast_multiplier
    }

    /// Create a config optimized for responsive player control.
    pub fn player() -> Self {
        Self::default()
    }

    /// Create a config for AI-controlled characters.
    pub fn ai() -> Self {
        Self {
            spring_strength: 300.0,
            spring_damping: 20.0,
            acceleration: 600.0,
            air_control: 0.1,
            ..default()
        }
    }

    /// Builder: set float height.
    pub fn with_float_height(mut self, height: f32) -> Self {
        self.float_height = height;
        self
    }

    /// Builder: set ground cast width.
    pub fn with_ground_cast_width(mut self, width: f32) -> Self {
        self.ground_cast_width = width;
        self
    }

    /// Builder: set spring parameters.
    pub fn with_spring(mut self, strength: f32, damping: f32) -> Self {
        self.spring_strength = strength;
        self.spring_damping = damping;
        self
    }

    /// Builder: set spring strength only (maintains damping ratio).
    pub fn with_spring_strength(mut self, strength: f32) -> Self {
        let ratio = self.spring_damping / self.spring_strength;
        self.spring_strength = strength;
        self.spring_damping = strength * ratio;
        self
    }

    /// Builder: set maximum spring force.
    /// Clamps the total spring force applied for floating.
    pub fn with_spring_max_force(mut self, max_force: f32) -> Self {
        self.spring_max_force = Some(max_force);
        self
    }

    /// Builder: set maximum velocity for spring force.
    /// If already moving toward the target at this speed or faster, no force is applied.
    pub fn with_spring_max_velocity(mut self, max_velocity: f32) -> Self {
        self.spring_max_velocity = Some(max_velocity);
        self
    }

    /// Builder: set movement parameters.
    pub fn with_movement(mut self, max_speed: f32, acceleration: f32) -> Self {
        self.max_speed = max_speed;
        self.acceleration = acceleration;
        self
    }

    /// Builder: set max speed.
    pub fn with_max_speed(mut self, max_speed: f32) -> Self {
        self.max_speed = max_speed;
        self
    }

    /// Builder: set coyote time.
    pub fn with_coyote_time(mut self, time: f32) -> Self {
        self.coyote_time = time;
        self
    }

    /// Builder: set jump buffer time.
    pub fn with_jump_buffer_time(mut self, time: f32) -> Self {
        self.jump_buffer_time = time;
        self
    }

    /// Builder: enable or disable upright torque.
    pub fn with_upright_torque_enabled(mut self, enabled: bool) -> Self {
        self.upright_torque_enabled = enabled;
        self
    }

    /// Builder: set upright torque parameters.
    pub fn with_upright_torque(mut self, strength: f32, damping: f32) -> Self {
        self.upright_torque_strength = strength;
        self.upright_torque_damping = damping;
        self
    }

    /// Builder: set jump speed.
    pub fn with_jump_speed(mut self, speed: f32) -> Self {
        self.jump_speed = speed;
        self
    }

    /// Builder: set fall gravity multiplier.
    pub fn with_fall_gravity(mut self, multiplier: f32) -> Self {
        self.fall_gravity = multiplier;
        self
    }

    /// Builder: set jump cancel window duration.
    /// Duration after jumping during which the jump can be cancelled.
    pub fn with_jump_cancel_window(mut self, duration: f32) -> Self {
        self.jump_cancel_window = duration;
        self
    }

    /// Builder: set fall gravity duration.
    /// Duration for which fall gravity is applied after cancellation.
    pub fn with_fall_gravity_duration(mut self, duration: f32) -> Self {
        self.fall_gravity_duration = duration;
        self
    }

    /// Builder: set upright target angle.
    pub fn with_upright_target_angle(mut self, angle: f32) -> Self {
        self.upright_target_angle = Some(angle);
        self
    }

    /// Builder: set maximum upright torque.
    /// Clamps the total torque applied for uprighting.
    pub fn with_upright_max_torque(mut self, max_torque: f32) -> Self {
        self.upright_max_torque = Some(max_torque);
        self
    }

    /// Builder: set maximum angular velocity for upright torque.
    /// If already rotating toward the target at this speed or faster, no torque is applied.
    pub fn with_upright_max_angular_velocity(mut self, max_velocity: f32) -> Self {
        self.upright_max_angular_velocity = Some(max_velocity);
        self
    }

    /// Builder: set jump spring filter duration.
    /// Duration after jump/upward propulsion during which downward spring forces are filtered.
    pub fn with_jump_spring_filter_duration(mut self, duration: f32) -> Self {
        self.jump_spring_filter_duration = duration;
        self
    }

    /// Builder: enable or disable wall jumping.
    pub fn with_wall_jumping(mut self, enabled: bool) -> Self {
        self.wall_jumping = enabled;
        self
    }

    /// Builder: set wall jump angle (radians from vertical).
    /// 0 = straight up, PI/4 (45°) = diagonal.
    pub fn with_wall_jump_angle(mut self, angle: f32) -> Self {
        self.wall_jump_angle = angle;
        self
    }

    /// Builder: set wall jump movement block duration.
    /// Duration (seconds) after a wall jump during which movement toward the wall is blocked.
    pub fn with_wall_jump_movement_block_duration(mut self, duration: f32) -> Self {
        self.wall_jump_movement_block_duration = duration;
        self
    }

    /// Builder: set wall jump velocity compensation (0.0-1.0).
    /// Controls how much downward velocity is cancelled before a wall jump.
    pub fn with_wall_jump_velocity_compensation(mut self, compensation: f32) -> Self {
        self.wall_jump_velocity_compensation = compensation.clamp(0.0, 1.0);
        self
    }

    /// Builder: set ground tolerance.
    pub fn with_ground_tolerance(mut self, tolerance: f32) -> Self {
        self.ground_tolerance = tolerance;
        self
    }

    /// Builder: set grounding distance.
    pub fn with_grounding_distance(mut self, distance: f32) -> Self {
        self.grounding_distance = distance;
        self
    }

    /// Builder: set grounding strength.
    pub fn with_grounding_strength(mut self, strength: f32) -> Self {
        self.grounding_strength = strength;
        self
    }
}

/// Configuration for stair stepping behavior.
///
/// The stair climbing system works by casting a ray downward in front of the character
/// (in the direction of movement intent) to detect steps. If a step is detected that is:
/// - Higher than the float_height (requires climbing)
/// - Lower than max_climb_height (climbable)
///
/// Then the system applies extra upward force and temporarily raises the riding height.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct StairConfig {
    /// Maximum step height the character can automatically climb.
    /// Steps higher than this will not be climbed.
    pub max_climb_height: f32,

    /// Minimum horizontal depth for a valid step.
    pub min_step_depth: f32,

    /// Width of the stair detection shapecast.
    pub stair_cast_width: f32,

    /// Offset from the collider radius for the stair detection cast.
    /// The cast origin is placed at: position + movement_direction * (radius + stair_cast_offset)
    /// Default is 2.0 pixels outside the collider radius.
    pub stair_cast_offset: f32,

    /// Tolerance for stair detection. Steps within this tolerance of the float height
    /// are not considered stairs (they're handled by the normal spring system).
    pub stair_tolerance: f32,

    /// Extra upward force multiplier when climbing stairs, as a multiple of max_spring_force.
    /// For example, 1.0 means apply the full max spring force as extra upward force.
    /// Using max_spring_force provides responsive climbing compared to gravity-based force.
    pub climb_force_multiplier: f32,

    /// Whether stair stepping is enabled.
    pub enabled: bool,
}

impl Default for StairConfig {
    fn default() -> Self {
        Self {
            max_climb_height: 8.0,
            min_step_depth: 4.0,
            stair_cast_width: 6.0,
            stair_cast_offset: 2.0,
            stair_tolerance: 1.0,
            climb_force_multiplier: 1.0,
            enabled: true,
        }
    }
}

impl StairConfig {
    /// Create a disabled stair config.
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..default()
        }
    }

    /// Builder: set max climb height.
    pub fn with_max_climb_height(mut self, height: f32) -> Self {
        self.max_climb_height = height;
        self
    }

    /// Builder: set stair cast width.
    pub fn with_stair_cast_width(mut self, width: f32) -> Self {
        self.stair_cast_width = width;
        self
    }

    /// Builder: set stair cast offset from collider radius.
    pub fn with_stair_cast_offset(mut self, offset: f32) -> Self {
        self.stair_cast_offset = offset;
        self
    }

    /// Builder: set stair tolerance.
    pub fn with_stair_tolerance(mut self, tolerance: f32) -> Self {
        self.stair_tolerance = tolerance;
        self
    }

    /// Builder: set climb force multiplier.
    pub fn with_climb_force_multiplier(mut self, multiplier: f32) -> Self {
        self.climb_force_multiplier = multiplier;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn controller_new() {
        let controller = CharacterController::new();
        let config = ControllerConfig::default();
        assert!(!controller.is_grounded(&config));
        assert_eq!(controller.gravity, Vec2::new(0.0, -980.0));
    }

    #[test]
    fn controller_with_gravity() {
        let gravity = Vec2::new(0.0, -500.0);
        let controller = CharacterController::with_gravity(gravity);
        assert_eq!(controller.gravity, gravity);
    }

    #[test]
    fn controller_touching_wall() {
        let mut controller = CharacterController::new();
        assert!(!controller.touching_wall());

        controller.left_wall = Some(CollisionData::new(1.0, Vec2::X, Vec2::ZERO, None));
        assert!(controller.touching_wall());
    }

    #[test]
    fn controller_is_grounded() {
        let config = ControllerConfig::default();
        let mut controller = CharacterController::new();
        controller.collider_bottom_offset = 10.0; // Simulate a capsule

        // No floor detected
        assert!(!controller.is_grounded(&config));

        // Floor detected within range
        let riding_height = config.float_height + controller.collider_bottom_offset;
        controller.floor = Some(CollisionData::new(riding_height, Vec2::Y, Vec2::ZERO, None));
        assert!(controller.is_grounded(&config));

        // Floor detected at edge of tolerance
        controller.floor = Some(CollisionData::new(
            riding_height + config.ground_tolerance,
            Vec2::Y,
            Vec2::ZERO,
            None,
        ));
        assert!(controller.is_grounded(&config));

        // Floor detected beyond tolerance
        controller.floor = Some(CollisionData::new(
            riding_height + config.ground_tolerance + 1.0,
            Vec2::Y,
            Vec2::ZERO,
            None,
        ));
        assert!(!controller.is_grounded(&config));
    }

    #[test]
    fn controller_in_spring_range() {
        let config = ControllerConfig::default();
        let mut controller = CharacterController::new();
        controller.collider_bottom_offset = 10.0;

        let riding_height = controller.riding_height(&config);

        // Floor at riding height - should be in range
        controller.floor = Some(CollisionData::new(riding_height, Vec2::Y, Vec2::ZERO, None));
        assert!(controller.in_spring_range(&config));

        // Floor below capsule half height - physics takes over
        controller.floor = Some(CollisionData::new(
            controller.collider_bottom_offset - 1.0,
            Vec2::Y,
            Vec2::ZERO,
            None,
        ));
        assert!(!controller.in_spring_range(&config));
    }

    #[test]
    fn config_derived_cast_lengths() {
        let config = ControllerConfig::default();

        // Wall cast derived from ground_cast_width
        assert_eq!(
            config.wall_cast_length(),
            config.ground_cast_width * config.wall_cast_multiplier
        );
    }

    #[test]
    fn config_player_preset() {
        let player = ControllerConfig::player();
        let default = ControllerConfig::default();
        assert!(player.spring_strength >= default.spring_strength);
    }

    #[test]
    fn stair_config_default() {
        let config = StairConfig::default();
        assert!(config.enabled);
        assert_eq!(config.max_climb_height, 8.0);
        assert_eq!(config.stair_cast_width, 6.0);
        assert_eq!(config.stair_cast_offset, 2.0);
        assert_eq!(config.stair_tolerance, 1.0);
        assert_eq!(config.climb_force_multiplier, 1.0);
    }

    #[test]
    fn stair_config_disabled() {
        let config = StairConfig::disabled();
        assert!(!config.enabled);
    }

    #[test]
    fn stair_config_builders() {
        let config = StairConfig::default()
            .with_max_climb_height(12.0)
            .with_stair_cast_width(8.0)
            .with_stair_cast_offset(3.0)
            .with_stair_tolerance(2.0)
            .with_climb_force_multiplier(3.0);

        assert_eq!(config.max_climb_height, 12.0);
        assert_eq!(config.stair_cast_width, 8.0);
        assert_eq!(config.stair_cast_offset, 3.0);
        assert_eq!(config.stair_tolerance, 2.0);
        assert_eq!(config.climb_force_multiplier, 3.0);
    }

    #[test]
    fn controller_effective_riding_height() {
        let config = ControllerConfig::default();
        let mut controller = CharacterController::new();
        controller.collider_bottom_offset = 10.0;

        // Without stair climbing
        let base_height = controller.riding_height(&config);
        assert_eq!(controller.effective_riding_height(&config), base_height);

        // With stair climbing
        controller.active_stair_height = 5.0;
        assert_eq!(controller.effective_riding_height(&config), base_height + 5.0);
    }

    #[test]
    fn controller_ideal_up_from_default_gravity() {
        let controller = CharacterController::new();
        // Default gravity is (0, -980), so ideal up should be (0, 1)
        let up = controller.ideal_up();
        assert!((up - Vec2::Y).length() < 0.001);
    }

    #[test]
    fn controller_ideal_directions_perpendicular() {
        let controller = CharacterController::new();
        let up = controller.ideal_up();
        let down = controller.ideal_down();
        let left = controller.ideal_left();
        let right = controller.ideal_right();

        // Up and down should be opposite
        assert!((up + down).length() < 0.001);
        // Left and right should be opposite
        assert!((left + right).length() < 0.001);
        // Up and right should be perpendicular
        assert!(up.dot(right).abs() < 0.001);
    }

    #[test]
    fn controller_ideal_up_with_custom_gravity() {
        // Gravity pointing left (sideways)
        let mut controller = CharacterController::new();
        controller.set_gravity(Vec2::new(-500.0, 0.0));

        // Ideal up should be opposite of gravity direction (pointing right)
        let up = controller.ideal_up();
        assert!((up - Vec2::X).length() < 0.001);

        // Right should be down in world terms
        let right = controller.ideal_right();
        assert!((right - Vec2::NEG_Y).length() < 0.001);
    }

    #[test]
    fn controller_ideal_up_with_zero_gravity() {
        let mut controller = CharacterController::new();
        controller.set_gravity(Vec2::ZERO);

        // With zero gravity, should default to Vec2::Y
        let up = controller.ideal_up();
        assert!((up - Vec2::Y).length() < 0.001);
    }

    #[test]
    fn controller_ideal_up_angle() {
        let controller = CharacterController::new();
        // Default gravity is down, so ideal up is Vec2::Y
        // Vec2::Y.to_angle() should be PI/2
        let angle = controller.ideal_up_angle();
        assert!((angle - FRAC_PI_2).abs() < 0.001);
    }
}
