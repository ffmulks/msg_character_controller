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
///
/// # Example
///
/// ```rust
/// use bevy::prelude::*;
/// use msg_character_controller::prelude::*;
///
/// // Default orientation (world up)
/// let orientation = CharacterOrientation::default();
/// assert_eq!(orientation.up(), Vec2::Y);
/// assert_eq!(orientation.down(), Vec2::NEG_Y);
///
/// // Custom orientation for walking on a ceiling
/// let ceiling = CharacterOrientation::new(Vec2::NEG_Y);
/// assert_eq!(ceiling.up(), Vec2::NEG_Y);
/// assert_eq!(ceiling.down(), Vec2::Y);
///
/// // Orientation for planetary gravity (pointing away from planet center)
/// fn update_planetary_orientation(
///     planet_center: Vec2,
///     mut query: Query<(&Transform, &mut CharacterOrientation)>,
/// ) {
///     for (transform, mut orientation) in &mut query {
///         let to_character = transform.translation.xy() - planet_center;
///         orientation.set_up(to_character.normalize_or_zero());
///     }
/// }
/// ```
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
    ///
    /// In 2D, this is the up vector rotated 90 degrees clockwise.
    #[inline]
    pub fn right(&self) -> Vec2 {
        Vec2::new(self.up.y, -self.up.x)
    }

    /// Get the "left" direction (perpendicular to up, counter-clockwise).
    ///
    /// In 2D, this is the up vector rotated 90 degrees counter-clockwise.
    #[inline]
    pub fn left(&self) -> Vec2 {
        Vec2::new(-self.up.y, self.up.x)
    }

    /// Set the "up" direction.
    ///
    /// The vector will be normalized. If zero-length, the orientation is unchanged.
    pub fn set_up(&mut self, up: Vec2) {
        let normalized = up.normalize_or_zero();
        if normalized != Vec2::ZERO {
            self.up = normalized;
        }
    }

    /// Create an orientation from an angle (radians from world +X axis).
    ///
    /// An angle of 0 means "up" points to the right (+X).
    /// An angle of PI/2 means "up" points to world up (+Y).
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
    ///
    /// Returns (horizontal, vertical) components where:
    /// - horizontal is positive rightward
    /// - vertical is positive upward
    pub fn to_local(&self, world_vec: Vec2) -> Vec2 {
        Vec2::new(world_vec.dot(self.right()), world_vec.dot(self.up))
    }

    /// Convert a local-space vector to world space.
    ///
    /// Takes (horizontal, vertical) and returns world coordinates.
    pub fn to_world(&self, local_vec: Vec2) -> Vec2 {
        self.right() * local_vec.x + self.up * local_vec.y
    }
}

/// Core character controller component.
///
/// Add this component to enable the floating character controller behavior.
/// The controller maintains a float height above ground using a spring-damper
/// system and handles movement based on intent components.
///
/// # Required Components
///
/// For the controller to function, the entity also needs:
/// - [`ControllerConfig`] - Controller tuning parameters
/// - Physics components from your backend (RigidBody, Collider, etc.)
/// - Movement intent components ([`super::intent::WalkIntent`] or [`super::intent::FlyIntent`])
///
/// # Optional Components
///
/// - [`super::intent::JumpRequest`] - For jumping capability
/// - [`StairConfig`] - For stair stepping (auto-added with defaults if missing)
#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct CharacterController {
    /// Current mode of the controller.
    pub mode: ControllerMode,
}

impl CharacterController {
    /// Create a new walking controller.
    pub fn walking() -> Self {
        Self {
            mode: ControllerMode::Walking,
        }
    }

    /// Create a new flying controller.
    pub fn flying() -> Self {
        Self {
            mode: ControllerMode::Flying,
        }
    }

    /// Check if the controller is in walking mode.
    pub fn is_walking(&self) -> bool {
        matches!(self.mode, ControllerMode::Walking)
    }

    /// Check if the controller is in flying mode.
    pub fn is_flying(&self) -> bool {
        matches!(self.mode, ControllerMode::Flying)
    }
}

/// Controller operating mode.
#[derive(Reflect, Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum ControllerMode {
    /// Ground-based movement with floating spring and gravity.
    #[default]
    Walking,
    /// Free 2D movement ignoring gravity.
    Flying,
}

/// Configuration parameters for the character controller.
///
/// These parameters control the physics behavior of the floating controller,
/// including spring strength, movement speeds, and slope handling.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct ControllerConfig {
    // === Float Settings ===
    /// Target height to float above ground (in world units/pixels).
    ///
    /// The controller will try to maintain this distance between the character
    /// and the ground surface.
    pub float_height: f32,

    /// Distance below float_height to still consider as "grounded".
    ///
    /// This provides a tolerance zone for ground detection. If the character
    /// is within `float_height + cling_distance` of ground, they're grounded.
    pub cling_distance: f32,

    /// Extra downward force multiplier when within cling distance.
    ///
    /// When the character is above float_height but within cling_distance,
    /// this multiplier applies additional downward force to help them
    /// "stick" to the ground on bumpy terrain or small ledges.
    /// 0.0 = no extra force, 1.0 = double gravity, etc.
    pub cling_strength: f32,

    // === Spring Settings ===
    /// Spring strength for the floating system.
    ///
    /// Higher values make the controller more "stiff" and responsive,
    /// but can cause oscillation if too high. Typical range: 100-1000.
    pub spring_strength: f32,

    /// Spring damping coefficient.
    ///
    /// Reduces oscillation in the spring system. Should be proportional
    /// to spring_strength. Typical range: 10-100.
    pub spring_damping: f32,

    // === Movement Settings ===
    /// Maximum horizontal movement speed (units/second).
    pub max_speed: f32,

    /// Horizontal acceleration rate (units/second^2).
    ///
    /// Controls how quickly the character reaches max speed.
    pub acceleration: f32,

    /// Friction/deceleration when no input (0.0-1.0).
    ///
    /// Applied as a velocity multiplier each physics step.
    /// 0.0 = no friction (ice), 1.0 = instant stop.
    pub friction: f32,

    /// Air control multiplier (0.0-1.0).
    ///
    /// Multiplies acceleration when airborne.
    pub air_control: f32,

    // === Slope Settings ===
    /// Maximum slope angle the character can walk up (radians).
    ///
    /// Slopes steeper than this will be treated as walls.
    /// PI/3 (~60 degrees) is a common value.
    pub max_slope_angle: f32,

    /// Extra downward force when walking uphill.
    ///
    /// Helps keep the character grounded on slopes.
    pub uphill_gravity_multiplier: f32,

    // === Shapecast Settings ===
    /// Length of the ground detection shapecast.
    ///
    /// Should be slightly longer than `float_height + cling_distance`.
    pub ground_cast_length: f32,

    /// Width of the ground detection shapecast.
    ///
    /// A wider cast provides better slope detection and stability on uneven
    /// terrain. Typical value is 50-100% of character width.
    pub ground_cast_width: f32,

    /// Length of wall detection shapecasts.
    pub wall_cast_length: f32,

    /// Width of wall detection shapecasts.
    pub wall_cast_width: f32,

    // === Jump Settings ===
    /// Jump impulse strength (applied as velocity).
    pub jump_speed: f32,

    /// Coyote time duration in seconds.
    ///
    /// Time after leaving ground where jump is still allowed.
    pub coyote_time: f32,

    /// Jump buffer duration in seconds.
    ///
    /// Time before landing where jump input is buffered.
    pub jump_buffer_time: f32,

    // === Upright Torque Settings ===
    /// Whether to apply torque to keep the character upright.
    ///
    /// When enabled, applies a spring-like torque force proportional to the
    /// square of the angle deviation from the target "up" direction.
    /// This creates a strong corrective force that increases rapidly as the
    /// character tilts further from upright.
    pub upright_torque_enabled: bool,

    /// Strength of the upright torque spring.
    ///
    /// The torque applied is: strength * angle_error² * sign(angle_error)
    /// Higher values create a stiffer response. Typical range: 50-500.
    pub upright_torque_strength: f32,

    /// Damping coefficient for the upright torque.
    ///
    /// Reduces oscillation by opposing angular velocity.
    /// Should be proportional to torque_strength. Typical range: 5-50.
    pub upright_torque_damping: f32,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            // Float settings
            float_height: 8.0,
            cling_distance: 2.0,
            cling_strength: 0.5,

            // Spring settings
            spring_strength: 400.0,
            spring_damping: 40.0,

            // Movement settings
            max_speed: 100.0,
            acceleration: 800.0,
            friction: 0.1,
            air_control: 0.3,

            // Slope settings
            max_slope_angle: std::f32::consts::FRAC_PI_3, // 60 degrees
            uphill_gravity_multiplier: 1.5,

            // Shapecast settings
            ground_cast_length: 12.0,
            ground_cast_width: 6.0,
            wall_cast_length: 6.0,
            wall_cast_width: 4.0,

            // Jump settings
            jump_speed: 200.0,
            coyote_time: 0.15,
            jump_buffer_time: 0.1,

            // Upright torque settings
            upright_torque_enabled: true,
            upright_torque_strength: 200.0,
            upright_torque_damping: 20.0,
        }
    }
}

impl ControllerConfig {
    /// Create a config optimized for responsive player control.
    pub fn player() -> Self {
        Self {
            spring_strength: 600.0,
            spring_damping: 60.0,
            acceleration: 1200.0,
            ..default()
        }
    }

    /// Create a config for AI-controlled characters.
    pub fn ai() -> Self {
        Self {
            spring_strength: 300.0,
            spring_damping: 30.0,
            acceleration: 600.0,
            air_control: 0.1,
            ..default()
        }
    }

    /// Create a config optimized for flying characters.
    pub fn flying() -> Self {
        Self {
            spring_strength: 0.0,
            spring_damping: 0.0,
            friction: 0.05,
            air_control: 1.0,
            upright_torque_enabled: false,
            ..default()
        }
    }

    /// Builder: set float height.
    pub fn with_float_height(mut self, height: f32) -> Self {
        self.float_height = height;
        self.ground_cast_length = height + self.cling_distance + 2.0;
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
}

/// Configuration for stair stepping behavior.
///
/// Stair stepping allows the character to automatically step over small
/// obstacles using additional raycasts to detect step-able geometry.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct StairConfig {
    /// Maximum step height the character can automatically climb.
    ///
    /// Obstacles higher than this will be treated as walls.
    pub max_step_height: f32,

    /// Minimum horizontal depth for a valid step.
    ///
    /// The top of the step must extend at least this far horizontally.
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
    use std::f32::consts::{FRAC_PI_2, PI};

    // ==================== CharacterOrientation Tests ====================

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

        let orientation = CharacterOrientation::new(Vec2::new(3.0, 4.0));
        assert!((orientation.up().length() - 1.0).abs() < 0.001);
    }

    #[test]
    fn orientation_new_handles_zero_vector() {
        let orientation = CharacterOrientation::new(Vec2::ZERO);
        assert_eq!(orientation.up(), Vec2::Y); // Falls back to default
    }

    #[test]
    fn orientation_ceiling_inverts_directions() {
        let orientation = CharacterOrientation::new(Vec2::NEG_Y);
        assert_eq!(orientation.up(), Vec2::NEG_Y);
        assert_eq!(orientation.down(), Vec2::Y);
        // Right and left are also inverted
        assert_eq!(orientation.right(), Vec2::NEG_X);
        assert_eq!(orientation.left(), Vec2::X);
    }

    #[test]
    fn orientation_sideways_walking() {
        // Walking on a wall where "up" points right
        let orientation = CharacterOrientation::new(Vec2::X);
        assert_eq!(orientation.up(), Vec2::X);
        assert_eq!(orientation.down(), Vec2::NEG_X);
        assert_eq!(orientation.right(), Vec2::NEG_Y);
        assert_eq!(orientation.left(), Vec2::Y);
    }

    #[test]
    fn orientation_set_up_updates_direction() {
        let mut orientation = CharacterOrientation::default();
        orientation.set_up(Vec2::NEG_Y);
        assert_eq!(orientation.up(), Vec2::NEG_Y);

        // Zero vector should not change orientation
        let old_up = orientation.up();
        orientation.set_up(Vec2::ZERO);
        assert_eq!(orientation.up(), old_up);
    }

    #[test]
    fn orientation_from_angle() {
        // 0 radians = pointing right (+X)
        let orientation = CharacterOrientation::from_angle(0.0);
        assert!((orientation.up() - Vec2::X).length() < 0.001);

        // PI/2 = pointing up (+Y)
        let orientation = CharacterOrientation::from_angle(FRAC_PI_2);
        assert!((orientation.up() - Vec2::Y).length() < 0.001);

        // PI = pointing left (-X)
        let orientation = CharacterOrientation::from_angle(PI);
        assert!((orientation.up() - Vec2::NEG_X).length() < 0.001);
    }

    #[test]
    fn orientation_angle_roundtrip() {
        for angle in [0.0, FRAC_PI_2, PI, -FRAC_PI_2, 0.5, 1.5, 2.5] {
            let orientation = CharacterOrientation::from_angle(angle);
            let recovered_angle = orientation.angle();
            // Angles can differ by 2*PI
            let diff = (angle - recovered_angle).abs() % (2.0 * PI);
            assert!(diff < 0.001 || (2.0 * PI - diff).abs() < 0.001);
        }
    }

    #[test]
    fn orientation_to_local_space() {
        let orientation = CharacterOrientation::default();

        // World right -> local right
        let local = orientation.to_local(Vec2::X);
        assert!((local - Vec2::X).length() < 0.001);

        // World up -> local up
        let local = orientation.to_local(Vec2::Y);
        assert!((local - Vec2::Y).length() < 0.001);

        // Diagonal
        let local = orientation.to_local(Vec2::new(1.0, 1.0));
        assert!((local - Vec2::new(1.0, 1.0)).length() < 0.001);
    }

    #[test]
    fn orientation_to_world_space() {
        let orientation = CharacterOrientation::default();

        // Local (1, 0) = world right
        let world = orientation.to_world(Vec2::X);
        assert!((world - Vec2::X).length() < 0.001);

        // Local (0, 1) = world up
        let world = orientation.to_world(Vec2::Y);
        assert!((world - Vec2::Y).length() < 0.001);
    }

    #[test]
    fn orientation_local_world_roundtrip() {
        let orientation = CharacterOrientation::new(Vec2::new(1.0, 1.0).normalize());

        for vec in [Vec2::X, Vec2::Y, Vec2::new(3.0, -4.0), Vec2::new(-1.0, 2.0)] {
            let local = orientation.to_local(vec);
            let back_to_world = orientation.to_world(local);
            assert!(
                (vec - back_to_world).length() < 0.001,
                "Failed roundtrip for {:?}",
                vec
            );
        }
    }

    #[test]
    fn orientation_planetary_gravity_simulation() {
        // Simulate a character on a planet, standing at angle 45 degrees
        let character_pos = Vec2::new(100.0, 100.0);
        let planet_center = Vec2::ZERO;
        let up = (character_pos - planet_center).normalize();

        let orientation = CharacterOrientation::new(up);

        // Down should point toward planet center
        let down = orientation.down();
        let expected_down = (planet_center - character_pos).normalize();
        assert!((down - expected_down).length() < 0.001);
    }

    // ==================== CharacterController Tests ====================

    #[test]
    fn controller_walking_mode() {
        let controller = CharacterController::walking();
        assert!(controller.is_walking());
        assert!(!controller.is_flying());
    }

    #[test]
    fn controller_flying_mode() {
        let controller = CharacterController::flying();
        assert!(controller.is_flying());
        assert!(!controller.is_walking());
    }

    // ==================== ControllerConfig Tests ====================

    #[test]
    fn config_default_values() {
        let config = ControllerConfig::default();
        assert_eq!(config.float_height, 8.0);
        assert!(config.spring_strength > 0.0);
        assert!(config.max_speed > 0.0);
        assert!(config.max_slope_angle > 0.0);
        assert!(config.coyote_time > 0.0);
        assert!(config.jump_buffer_time > 0.0);
    }

    #[test]
    fn config_player_preset() {
        let player = ControllerConfig::player();
        let default = ControllerConfig::default();

        // Player should have higher responsiveness
        assert!(player.spring_strength >= default.spring_strength);
        assert!(player.acceleration >= default.acceleration);
    }

    #[test]
    fn config_ai_preset() {
        let ai = ControllerConfig::ai();
        let default = ControllerConfig::default();

        // AI should have lower air control
        assert!(ai.air_control <= default.air_control);
    }

    #[test]
    fn config_flying_preset() {
        let flying = ControllerConfig::flying();

        // Flying should have no spring (floats freely)
        assert_eq!(flying.spring_strength, 0.0);
        assert_eq!(flying.spring_damping, 0.0);
        // Full air control
        assert_eq!(flying.air_control, 1.0);
    }

    #[test]
    fn config_builder_pattern() {
        let config = ControllerConfig::default()
            .with_float_height(16.0)
            .with_spring(500.0, 50.0)
            .with_movement(200.0, 1000.0)
            .with_ground_cast_width(8.0);

        assert_eq!(config.float_height, 16.0);
        assert_eq!(config.spring_strength, 500.0);
        assert_eq!(config.spring_damping, 50.0);
        assert_eq!(config.max_speed, 200.0);
        assert_eq!(config.acceleration, 1000.0);
        assert_eq!(config.ground_cast_width, 8.0);
    }

    #[test]
    fn config_with_float_height_updates_cast_length() {
        let config = ControllerConfig::default().with_float_height(20.0);
        // Cast length should be at least float_height + cling_distance
        assert!(config.ground_cast_length >= config.float_height + config.cling_distance);
    }

    // ==================== StairConfig Tests ====================

    #[test]
    fn stair_config_default() {
        let config = StairConfig::default();
        assert!(config.enabled);
        assert!(config.max_step_height > 0.0);
    }

    #[test]
    fn stair_config_disabled() {
        let config = StairConfig::disabled();
        assert!(!config.enabled);
    }

    #[test]
    fn stair_config_builder() {
        let config = StairConfig::default().with_max_height(12.0);
        assert_eq!(config.max_step_height, 12.0);
    }
}
