//! Controller configuration components.
//!
//! This module defines the core configuration for character controllers,
//! including float height, spring parameters, slope limits, and stair stepping.

use bevy::prelude::*;

use crate::collision::CollisionData;

/// Defines the local coordinate system for a character controller.
///
/// This component allows characters to orient themselves relative to arbitrary
/// "up" directions, enabling walking on rotating platforms, around planets,
/// or on walls/ceilings.
///
/// The orientation is defined by a single `up` vector. The horizontal axes
/// (left/right for walking, full 2D for flying) are derived perpendicular to `up`.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct CharacterOrientation {
    /// The "up" direction for this character.
    up: Vec2,
}

impl Default for CharacterOrientation {
    fn default() -> Self {
        Self { up: Vec2::Y }
    }
}

impl CharacterOrientation {
    /// Create a new orientation with the given up direction.
    ///
    /// The vector will be normalized. If zero-length, defaults to `Vec2::Y`.
    pub fn new(up: Vec2) -> Self {
        let normalized = up.normalize_or_zero();
        Self {
            up: if normalized == Vec2::ZERO {
                Vec2::Y
            } else {
                normalized
            },
        }
    }

    /// Get the "up" direction.
    #[inline]
    pub fn up(&self) -> Vec2 {
        self.up
    }

    /// Get the "down" direction (opposite of up).
    #[inline]
    pub fn down(&self) -> Vec2 {
        -self.up
    }

    /// Get the "right" direction (perpendicular to up, clockwise).
    #[inline]
    pub fn right(&self) -> Vec2 {
        Vec2::new(self.up.y, -self.up.x)
    }

    /// Get the "left" direction (perpendicular to up, counter-clockwise).
    #[inline]
    pub fn left(&self) -> Vec2 {
        Vec2::new(-self.up.y, self.up.x)
    }

    /// Set the "up" direction.
    pub fn set_up(&mut self, up: Vec2) {
        let normalized = up.normalize_or_zero();
        if normalized != Vec2::ZERO {
            self.up = normalized;
        }
    }

    /// Create an orientation from an angle (radians from world +X axis).
    pub fn from_angle(angle: f32) -> Self {
        Self {
            up: Vec2::from_angle(angle),
        }
    }

    /// Get the angle of the up direction (radians from world +X axis).
    pub fn angle(&self) -> f32 {
        self.up.to_angle()
    }

    /// Project a world-space vector into this orientation's local space.
    pub fn to_local(&self, world_vec: Vec2) -> Vec2 {
        Vec2::new(world_vec.dot(self.right()), world_vec.dot(self.up))
    }

    /// Convert a local-space vector to world space.
    pub fn to_world(&self, local_vec: Vec2) -> Vec2 {
        self.right() * local_vec.x + self.up * local_vec.y
    }
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
#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
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
    /// Time since last grounded (for coyote time).
    pub time_since_grounded: f32,

    // === Gravity ===
    /// Gravity vector affecting this character.
    /// Used for floating spring, extra fall gravity, and jump countering.
    pub gravity: Vec2,

    // === Spring State ===
    /// Whether the character left the spring range after a jump (for spring exception).
    /// Reset when entering spring range again from above.
    pub(crate) left_spring_range_after_jump: bool,

    // === Internal (used by systems, kept pub(crate)) ===
    /// Distance from collider center to bottom (auto-detected from Collider).
    /// For a capsule, this is half_height + radius.
    pub(crate) collider_bottom_offset: f32,
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
            time_since_grounded: 0.0,
            // Gravity
            gravity: Vec2::new(0.0, -980.0),
            // Spring state
            left_spring_range_after_jump: false,
            // Internal
            collider_bottom_offset: 0.0,
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
        Self { gravity, ..default() }
    }

    /// Set the gravity vector.
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.gravity = gravity;
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
    }

    /// Get the riding height (desired height from ground to collider center).
    /// This is float_height + collider_bottom_offset (half_height + radius for capsule).
    #[inline]
    pub fn riding_height(&self, config: &ControllerConfig) -> f32 {
        config.float_height + self.collider_bottom_offset
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
    pub cling_distance: f32,

    /// Extra downward force multiplier when within cling distance.
    pub cling_strength: f32,

    // === Spring Settings ===
    /// Spring strength for the floating system.
    pub spring_strength: f32,

    /// Spring damping coefficient.
    pub spring_damping: f32,

    // === Movement Settings ===
    /// Maximum horizontal movement speed (units/second).
    pub max_speed: f32,

    /// Horizontal acceleration rate (units/second^2).
    pub acceleration: f32,

    /// Friction/deceleration when no input (0.0-1.0).
    pub friction: f32,

    /// Air control multiplier (0.0-1.0).
    pub air_control: f32,

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

    // === Mass Settings ===
    /// Reference mass for force calculations.
    ///
    /// When `Some(mass)`, all force-based parameters (spring_strength, jump_speed,
    /// upright_torque_strength) are scaled proportionally based on the actual
    /// rigid body mass to produce consistent acceleration/velocity.
    ///
    /// When `None`, forces are applied using the actual rigid body mass directly
    /// (no scaling). This is the recommended default as it works automatically
    /// with Rapier's mass computed from collider geometry.
    ///
    /// For example, if `mass = Some(1.0)` and `jump_speed = 300.0`, a character
    /// with actual mass 10.0 will receive an impulse of 3000.0 to achieve the
    /// same jump velocity.
    pub mass: Option<f32>,

    // === Jump Settings ===
    /// Jump impulse strength (applied as velocity).
    pub jump_speed: f32,

    /// Coyote time duration in seconds.
    pub coyote_time: f32,

    /// Jump buffer duration in seconds.
    pub jump_buffer_time: f32,

    /// Extra gravity multiplier when falling.
    pub extra_fall_gravity: f32,

    // === Upright Torque Settings ===
    /// Whether to apply torque to keep the character upright.
    pub upright_torque_enabled: bool,

    /// Strength of the upright torque spring.
    pub upright_torque_strength: f32,

    /// Damping coefficient for the upright torque.
    pub upright_torque_damping: f32,

    /// Target angle for upright torque (radians). None = use CharacterOrientation.
    pub upright_target_angle: Option<f32>,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            // Float settings
            float_height: 1.0, // 1 pixel gap between collider bottom and ground
            ground_tolerance: 2.0, // tolerance for spring activation
            cling_distance: 2.0, // distance for wall/ceiling detection
            cling_strength: 0.5,

            // Spring settings (tuned for mass=1.0)
            spring_strength: 500.0,
            spring_damping: 30.0,

            // Movement settings
            max_speed: 100.0,
            acceleration: 800.0,
            friction: 0.1,
            air_control: 0.3,

            // Slope settings
            max_slope_angle: std::f32::consts::FRAC_PI_3, // 60 degrees
            uphill_gravity_multiplier: 1.0,

            // Sensor settings (derived from float_height)
            ground_cast_multiplier: 10.0,
            ground_cast_width: 6.0,
            wall_cast_multiplier: 1.5,
            wall_cast_height: 4.0,
            ceiling_cast_multiplier: 2.0,
            ceiling_cast_width: 6.0,

            // Mass setting (None = use actual Rapier mass)
            mass: None,

            // Jump settings
            jump_speed: 300.0,
            coyote_time: 0.15,
            jump_buffer_time: 0.1,
            extra_fall_gravity: 1.0,

            // Upright torque settings
            upright_torque_enabled: true,
            upright_torque_strength: 200.0,
            upright_torque_damping: 20.0,
            upright_target_angle: None,
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
        Self {
            spring_strength: 800.0,
            spring_damping: 50.0,
            acceleration: 1200.0,
            jump_speed: 350.0,
            ..default()
        }
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

    /// Builder: set extra fall gravity multiplier.
    pub fn with_extra_fall_gravity(mut self, multiplier: f32) -> Self {
        self.extra_fall_gravity = multiplier;
        self
    }

    /// Builder: set upright target angle.
    pub fn with_upright_target_angle(mut self, angle: f32) -> Self {
        self.upright_target_angle = Some(angle);
        self
    }

    /// Builder: set ground tolerance.
    pub fn with_ground_tolerance(mut self, tolerance: f32) -> Self {
        self.ground_tolerance = tolerance;
        self
    }

    /// Builder: set cling distance.
    pub fn with_cling_distance(mut self, distance: f32) -> Self {
        self.cling_distance = distance;
        self
    }

    /// Builder: set reference mass.
    ///
    /// When set, forces will be scaled so that the config parameters produce
    /// consistent behavior regardless of actual mass. This is useful when you
    /// want the same config values to work across characters of different sizes.
    ///
    /// Leave as `None` (default) to use actual Rapier mass directly.
    pub fn with_mass(mut self, mass: f32) -> Self {
        self.mass = Some(mass);
        self
    }
}

/// Configuration for stair stepping behavior.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct StairConfig {
    /// Maximum step height the character can automatically climb.
    pub max_step_height: f32,

    /// Minimum horizontal depth for a valid step.
    pub min_step_depth: f32,

    /// Forward raycast distance for step detection.
    pub step_check_distance: f32,

    /// Whether stair stepping is enabled.
    pub enabled: bool,
}

impl Default for StairConfig {
    fn default() -> Self {
        Self {
            max_step_height: 8.0,
            min_step_depth: 4.0,
            step_check_distance: 8.0,
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

    /// Builder: set max step height.
    pub fn with_max_height(mut self, height: f32) -> Self {
        self.max_step_height = height;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn orientation_default_is_world_up() {
        let orientation = CharacterOrientation::default();
        assert_eq!(orientation.up(), Vec2::Y);
        assert_eq!(orientation.down(), Vec2::NEG_Y);
        assert_eq!(orientation.right(), Vec2::X);
        assert_eq!(orientation.left(), Vec2::NEG_X);
    }

    #[test]
    fn orientation_new_normalizes_input() {
        let orientation = CharacterOrientation::new(Vec2::new(0.0, 10.0));
        assert!((orientation.up() - Vec2::Y).length() < 0.001);
    }

    #[test]
    fn orientation_from_angle() {
        let orientation = CharacterOrientation::from_angle(FRAC_PI_2);
        assert!((orientation.up() - Vec2::Y).length() < 0.001);
    }

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
        controller.floor = Some(CollisionData::new(riding_height + config.ground_tolerance, Vec2::Y, Vec2::ZERO, None));
        assert!(controller.is_grounded(&config));

        // Floor detected beyond tolerance
        controller.floor = Some(CollisionData::new(riding_height + config.ground_tolerance + 1.0, Vec2::Y, Vec2::ZERO, None));
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
        controller.floor = Some(CollisionData::new(controller.collider_bottom_offset - 1.0, Vec2::Y, Vec2::ZERO, None));
        assert!(!controller.in_spring_range(&config));
    }

    #[test]
    fn config_derived_cast_lengths() {
        let config = ControllerConfig::default();

        // Wall cast derived from ground_cast_width
        assert_eq!(config.wall_cast_length(), config.ground_cast_width * config.wall_cast_multiplier);
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
    }

    #[test]
    fn stair_config_disabled() {
        let config = StairConfig::disabled();
        assert!(!config.enabled);
    }
}
