//! Controller configuration components.
//!
//! This module defines the core configuration for character controllers,
//! including float height, spring parameters, slope limits, and stair stepping.

use bevy::prelude::*;

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
/// - `is_grounded`: True when standing on walkable ground (within float_height)
/// - `touching_left_wall`: True when touching a wall on the left
/// - `touching_right_wall`: True when touching a wall on the right
/// - `touching_ceiling`: True when touching a ceiling above
#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
pub struct CharacterController {
    // === Ground Contact State (RESULT) ===
    /// Whether the character is grounded (standing on walkable ground).
    /// True when ground is detected within `float_height` from the character.
    pub is_grounded: bool,
    /// Ground surface normal (points away from surface). Valid when is_grounded.
    pub ground_normal: Vec2,
    /// Slope angle in radians (0 = flat). Valid when is_grounded.
    pub slope_angle: f32,
    /// Whether a valid step was detected (for stair stepping).
    pub step_detected: bool,
    /// Height of the detected step (if any).
    pub step_height: f32,
    /// Time since last grounded (for coyote time).
    pub time_since_grounded: f32,
    /// Entity that is the ground. Valid when is_grounded.
    pub ground_entity: Option<Entity>,

    // === Wall Contact State (RESULTS) ===
    /// Whether touching a wall on the left side.
    pub touching_left_wall: bool,
    /// Left wall normal. Valid when touching_left_wall.
    pub left_wall_normal: Vec2,
    /// Whether touching a wall on the right side.
    pub touching_right_wall: bool,
    /// Right wall normal. Valid when touching_right_wall.
    pub right_wall_normal: Vec2,

    // === Ceiling Contact State (RESULT) ===
    /// Whether touching a ceiling above.
    pub touching_ceiling: bool,
    /// Ceiling surface normal. Valid when touching_ceiling.
    pub ceiling_normal: Vec2,

    // === Gravity ===
    /// Gravity vector affecting this character.
    /// Used for floating spring, extra fall gravity, and jump countering.
    pub gravity: Vec2,

    // === Internal (used by systems, kept pub(crate)) ===
    /// Raw distance to ground (internal use for spring calculations).
    pub(crate) ground_distance: f32,
    /// Ground contact point in world space (internal use).
    pub(crate) ground_contact_point: Vec2,
    /// Whether ground raycast hit something (may be further than float_height).
    pub(crate) ground_detected: bool,
}

impl Default for CharacterController {
    fn default() -> Self {
        Self {
            // Ground contact (RESULT)
            is_grounded: false,
            ground_normal: Vec2::Y,
            slope_angle: 0.0,
            step_detected: false,
            step_height: 0.0,
            time_since_grounded: 0.0,
            ground_entity: None,
            // Wall contact (RESULTS)
            touching_left_wall: false,
            left_wall_normal: Vec2::X,
            touching_right_wall: false,
            right_wall_normal: Vec2::NEG_X,
            // Ceiling contact (RESULT)
            touching_ceiling: false,
            ceiling_normal: Vec2::NEG_Y,
            // Gravity
            gravity: Vec2::new(0.0, -980.0),
            // Internal
            ground_distance: f32::MAX,
            ground_contact_point: Vec2::ZERO,
            ground_detected: false,
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

    /// Get the ground tangent vector (for movement direction along slopes).
    pub fn ground_tangent(&self) -> Vec2 {
        Vec2::new(self.ground_normal.y, -self.ground_normal.x)
    }

    /// Check if touching any wall.
    pub fn touching_wall(&self) -> bool {
        self.touching_left_wall || self.touching_right_wall
    }

    /// Get the wall normal if touching a wall in the given direction.
    pub fn wall_normal(&self, direction: f32) -> Option<Vec2> {
        if direction < 0.0 && self.touching_left_wall {
            Some(self.left_wall_normal)
        } else if direction > 0.0 && self.touching_right_wall {
            Some(self.right_wall_normal)
        } else {
            None
        }
    }

    /// Check if on a slope that requires extra handling.
    pub fn is_on_slope(&self) -> bool {
        self.is_grounded && self.slope_angle.abs() > 0.1
    }

    /// Get the raw distance to ground (for debugging/testing).
    ///
    /// This returns the raycast hit distance, which may be beyond float_height.
    /// Use `is_grounded` for gameplay logic instead.
    pub fn ground_distance(&self) -> f32 {
        self.ground_distance
    }

    /// Check if ground was detected by raycast (for debugging/testing).
    ///
    /// This returns true if the raycast hit something, even if beyond float_height.
    /// Use `is_grounded` for gameplay logic instead.
    pub fn ground_detected(&self) -> bool {
        self.ground_detected
    }

    /// Get the ground contact point in world space (for debugging/testing).
    pub fn ground_contact_point(&self) -> Vec2 {
        self.ground_contact_point
    }

    /// Reset all detection state (called at start of each frame).
    pub(crate) fn reset_detection_state(&mut self) {
        // Ground
        self.is_grounded = false;
        self.ground_detected = false;
        self.ground_distance = f32::MAX;
        self.ground_normal = Vec2::Y;
        self.ground_contact_point = Vec2::ZERO;
        self.slope_angle = 0.0;
        self.step_detected = false;
        self.step_height = 0.0;
        self.ground_entity = None;

        // Walls
        self.touching_left_wall = false;
        self.left_wall_normal = Vec2::X;
        self.touching_right_wall = false;
        self.right_wall_normal = Vec2::NEG_X;

        // Ceiling
        self.touching_ceiling = false;
        self.ceiling_normal = Vec2::NEG_Y;
    }

    /// Calculate height error for floating spring (internal use).
    pub(crate) fn height_error(&self, target_height: f32) -> f32 {
        if self.ground_detected {
            target_height - self.ground_distance
        } else {
            0.0
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
    /// This is the key setting - other sensor lengths are derived from it.
    pub float_height: f32,

    /// Extra distance beyond float_height to still consider as "grounded".
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
            float_height: 15.0,
            cling_distance: 2.0,
            cling_strength: 0.5,

            // Spring settings
            spring_strength: 8000.0,
            spring_damping: 200.0,

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

            // Jump settings
            jump_speed: 5000.0,
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
    /// Get the ground cast length (derived from float_height).
    #[inline]
    pub fn ground_cast_length(&self) -> f32 {
        self.float_height * self.ground_cast_multiplier
    }

    /// Get the wall cast length (derived from ground_cast_width).
    #[inline]
    pub fn wall_cast_length(&self) -> f32 {
        self.ground_cast_width * self.wall_cast_multiplier
    }

    /// Get the ceiling cast length (derived from float_height).
    #[inline]
    pub fn ceiling_cast_length(&self) -> f32 {
        self.float_height * self.ceiling_cast_multiplier
    }

    /// Create a config optimized for responsive player control.
    pub fn player() -> Self {
        Self {
            spring_strength: 12000.0,
            spring_damping: 300.0,
            acceleration: 1200.0,
            ..default()
        }
    }

    /// Create a config for AI-controlled characters.
    pub fn ai() -> Self {
        Self {
            spring_strength: 1500.0,
            spring_damping: 75.0,
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
        assert!(!controller.is_grounded);
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

        controller.touching_left_wall = true;
        assert!(controller.touching_wall());
    }

    #[test]
    fn config_derived_cast_lengths() {
        let config = ControllerConfig::default().with_float_height(20.0);

        // Ground cast derived from float_height
        assert_eq!(config.ground_cast_length(), 20.0 * config.ground_cast_multiplier);

        // Wall cast derived from ground_cast_width
        assert_eq!(config.wall_cast_length(), config.ground_cast_width * config.wall_cast_multiplier);

        // Ceiling cast derived from float_height
        assert_eq!(config.ceiling_cast_length(), 20.0 * config.ceiling_cast_multiplier);
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
