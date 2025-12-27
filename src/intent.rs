//! Movement intent components.
//!
//! Intents represent the desired movement direction from player input or AI.
//! The controller systems read these intents and apply appropriate physics.

use bevy::prelude::*;

/// Walking movement intent (1D horizontal axis).
///
/// For ground-based characters, this controls horizontal movement along
/// the ground plane. The value should be in the range -1.0 to 1.0.
///
/// # Example
///
/// ```rust
/// use msg_character_controller::prelude::*;
///
/// // Create intent moving right
/// let mut intent = WalkIntent::new(1.0);
/// assert!(intent.is_active());
/// assert_eq!(intent.effective(), 1.0);
///
/// // Apply half speed
/// intent.set_speed(0.5);
/// assert_eq!(intent.effective(), 0.5);
///
/// // Clear to stop
/// intent.clear();
/// assert!(!intent.is_active());
/// ```
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct WalkIntent {
    /// Horizontal movement intent (-1.0 = left, 1.0 = right).
    pub direction: f32,
    /// Speed multiplier (0.0 to 1.0).
    pub speed_multiplier: f32,
}

impl Default for WalkIntent {
    fn default() -> Self {
        Self {
            direction: 0.0,
            speed_multiplier: 1.0, // Default to full speed so set() works immediately
        }
    }
}

impl WalkIntent {
    /// Create a new walk intent with the given direction.
    pub fn new(direction: f32) -> Self {
        Self {
            direction: direction.clamp(-1.0, 1.0),
            speed_multiplier: 1.0,
        }
    }

    /// Set the movement direction.
    pub fn set(&mut self, direction: f32) {
        self.direction = direction.clamp(-1.0, 1.0);
    }

    /// Set the speed multiplier (for walk vs run).
    pub fn set_speed(&mut self, multiplier: f32) {
        self.speed_multiplier = multiplier.clamp(0.0, 1.0);
    }

    /// Clear the intent (stop moving).
    pub fn clear(&mut self) {
        self.direction = 0.0;
    }

    /// Check if there is active input.
    pub fn is_active(&self) -> bool {
        self.direction.abs() > 0.001
    }

    /// Get the effective direction with speed multiplier applied.
    pub fn effective(&self) -> f32 {
        self.direction * self.speed_multiplier
    }
}

/// Flying movement intent (2D axis).
///
/// For flying characters, this controls movement in both horizontal and
/// vertical axes. Values should be in the range -1.0 to 1.0 for each axis.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct FlyIntent {
    /// Movement direction (x = horizontal, y = vertical).
    pub direction: Vec2,
    /// Speed multiplier (0.0 to 1.0).
    pub speed_multiplier: f32,
}

impl Default for FlyIntent {
    fn default() -> Self {
        Self {
            direction: Vec2::ZERO,
            speed_multiplier: 1.0, // Default to full speed so set() works immediately
        }
    }
}

impl FlyIntent {
    /// Create a new fly intent with the given direction.
    pub fn new(direction: Vec2) -> Self {
        Self {
            direction: direction.clamp(Vec2::NEG_ONE, Vec2::ONE),
            speed_multiplier: 1.0,
        }
    }

    /// Set the movement direction.
    pub fn set(&mut self, direction: Vec2) {
        self.direction = direction.clamp(Vec2::NEG_ONE, Vec2::ONE);
    }

    /// Set just the horizontal component.
    pub fn set_horizontal(&mut self, x: f32) {
        self.direction.x = x.clamp(-1.0, 1.0);
    }

    /// Set just the vertical component.
    pub fn set_vertical(&mut self, y: f32) {
        self.direction.y = y.clamp(-1.0, 1.0);
    }

    /// Set the speed multiplier.
    pub fn set_speed(&mut self, multiplier: f32) {
        self.speed_multiplier = multiplier.clamp(0.0, 1.0);
    }

    /// Clear the intent (stop moving).
    pub fn clear(&mut self) {
        self.direction = Vec2::ZERO;
    }

    /// Check if there is active input.
    pub fn is_active(&self) -> bool {
        self.direction.length_squared() > 0.001
    }

    /// Get the effective direction with speed multiplier applied.
    pub fn effective(&self) -> Vec2 {
        self.direction * self.speed_multiplier
    }
}

/// Jump request component.
///
/// Add this component to request a jump. The controller will consume
/// this request and attempt to execute a jump if conditions allow
/// (grounded, within coyote time, etc.).
#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct JumpRequest {
    /// Whether a jump is currently requested.
    pub requested: bool,
    /// Time the request was made (for jump buffering).
    pub request_time: f32,
    /// Whether this request has been consumed.
    pub consumed: bool,
}

impl JumpRequest {
    /// Request a jump.
    pub fn request(&mut self, current_time: f32) {
        if !self.requested {
            self.requested = true;
            self.request_time = current_time;
            self.consumed = false;
        }
    }

    /// Check if the request is valid (not consumed and within buffer time).
    pub fn is_valid(&self, current_time: f32, buffer_time: f32) -> bool {
        self.requested && !self.consumed && (current_time - self.request_time) < buffer_time
    }

    /// Consume the jump request.
    pub fn consume(&mut self) {
        self.consumed = true;
    }

    /// Reset the request.
    pub fn reset(&mut self) {
        self.requested = false;
        self.consumed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== WalkIntent Tests ====================

    #[test]
    fn walk_intent_new() {
        let intent = WalkIntent::new(0.5);
        assert_eq!(intent.direction, 0.5);
        assert_eq!(intent.speed_multiplier, 1.0);
    }

    #[test]
    fn walk_intent_clamps_direction() {
        let intent = WalkIntent::new(5.0);
        assert_eq!(intent.direction, 1.0);

        let intent = WalkIntent::new(-5.0);
        assert_eq!(intent.direction, -1.0);
    }

    #[test]
    fn walk_intent_set() {
        let mut intent = WalkIntent::default();
        intent.set(0.75);
        assert_eq!(intent.direction, 0.75);

        intent.set(2.0);
        assert_eq!(intent.direction, 1.0);
    }

    #[test]
    fn walk_intent_speed_multiplier() {
        let mut intent = WalkIntent::new(1.0);
        intent.set_speed(0.5);
        assert_eq!(intent.effective(), 0.5);

        // Clamps to valid range
        intent.set_speed(2.0);
        assert_eq!(intent.speed_multiplier, 1.0);

        intent.set_speed(-1.0);
        assert_eq!(intent.speed_multiplier, 0.0);
    }

    #[test]
    fn walk_intent_is_active() {
        let mut intent = WalkIntent::default();
        assert!(!intent.is_active());

        intent.set(0.5);
        assert!(intent.is_active());

        intent.set(0.0001); // Below threshold
        assert!(!intent.is_active());
    }

    #[test]
    fn walk_intent_clear() {
        let mut intent = WalkIntent::new(1.0);
        assert!(intent.is_active());

        intent.clear();
        assert!(!intent.is_active());
        assert_eq!(intent.direction, 0.0);
    }

    #[test]
    fn walk_intent_effective() {
        let mut intent = WalkIntent::new(-1.0);
        intent.set_speed(0.5);
        assert_eq!(intent.effective(), -0.5);
    }

    // ==================== FlyIntent Tests ====================

    #[test]
    fn fly_intent_new() {
        let intent = FlyIntent::new(Vec2::new(0.5, -0.5));
        assert_eq!(intent.direction, Vec2::new(0.5, -0.5));
        assert_eq!(intent.speed_multiplier, 1.0);
    }

    #[test]
    fn fly_intent_clamps_direction() {
        let intent = FlyIntent::new(Vec2::new(5.0, -5.0));
        assert_eq!(intent.direction, Vec2::new(1.0, -1.0));
    }

    #[test]
    fn fly_intent_set_horizontal() {
        let mut intent = FlyIntent::default();
        intent.set_horizontal(0.8);
        assert_eq!(intent.direction.x, 0.8);
        assert_eq!(intent.direction.y, 0.0);
    }

    #[test]
    fn fly_intent_set_vertical() {
        let mut intent = FlyIntent::default();
        intent.set_vertical(-0.6);
        assert_eq!(intent.direction.y, -0.6);
        assert_eq!(intent.direction.x, 0.0);
    }

    #[test]
    fn fly_intent_is_active() {
        let mut intent = FlyIntent::default();
        assert!(!intent.is_active());

        intent.set(Vec2::new(0.5, 0.5));
        assert!(intent.is_active());

        intent.set(Vec2::new(0.0001, 0.0001)); // Below threshold
        assert!(!intent.is_active());
    }

    #[test]
    fn fly_intent_clear() {
        let mut intent = FlyIntent::new(Vec2::ONE);
        assert!(intent.is_active());

        intent.clear();
        assert!(!intent.is_active());
        assert_eq!(intent.direction, Vec2::ZERO);
    }

    #[test]
    fn fly_intent_effective() {
        let mut intent = FlyIntent::new(Vec2::new(1.0, -1.0));
        intent.set_speed(0.5);
        assert_eq!(intent.effective(), Vec2::new(0.5, -0.5));
    }

    // ==================== JumpRequest Tests ====================

    #[test]
    fn jump_request_default() {
        let request = JumpRequest::default();
        assert!(!request.requested);
        assert!(!request.consumed);
    }

    #[test]
    fn jump_request_request() {
        let mut request = JumpRequest::default();
        request.request(1.0);

        assert!(request.requested);
        assert!(!request.consumed);
        assert_eq!(request.request_time, 1.0);
    }

    #[test]
    fn jump_request_only_requests_once() {
        let mut request = JumpRequest::default();
        request.request(1.0);
        request.request(2.0); // Should not update time

        assert_eq!(request.request_time, 1.0);
    }

    #[test]
    fn jump_request_is_valid() {
        let mut request = JumpRequest::default();
        request.request(1.0);

        // Within buffer time
        assert!(request.is_valid(1.05, 0.1));

        // Outside buffer time
        assert!(!request.is_valid(1.2, 0.1));
    }

    #[test]
    fn jump_request_consume() {
        let mut request = JumpRequest::default();
        request.request(1.0);
        assert!(request.is_valid(1.0, 0.1));

        request.consume();
        assert!(!request.is_valid(1.0, 0.1));
    }

    #[test]
    fn jump_request_reset() {
        let mut request = JumpRequest::default();
        request.request(1.0);
        request.consume();

        request.reset();
        assert!(!request.requested);
        assert!(!request.consumed);
    }

    #[test]
    fn jump_request_buffer_edge_case() {
        let mut request = JumpRequest::default();
        request.request(1.0);

        // Exactly at buffer time boundary
        assert!(!request.is_valid(1.1, 0.1));
    }
}
