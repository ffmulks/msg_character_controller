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

/// Vertical propulsion intent (for jetpacks, thrusters, etc.).
///
/// This controls vertical propulsion independently from walking.
/// Positive values thrust upward, negative values thrust downward.
/// When thrusting upward, the propulsion is automatically strengthened
/// by the magnitude of gravity to help counteract it.
///
/// # Example
///
/// ```rust
/// use msg_character_controller::prelude::*;
///
/// // Create intent thrusting upward
/// let mut intent = PropulsionIntent::new(1.0);
/// assert!(intent.is_active());
///
/// // Thrust downward
/// intent.set(-1.0);
/// assert_eq!(intent.direction, -1.0);
///
/// // Clear to stop
/// intent.clear();
/// assert!(!intent.is_active());
/// ```
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct PropulsionIntent {
    /// Vertical propulsion intent (-1.0 = down, 1.0 = up).
    pub direction: f32,
    /// Speed multiplier (0.0 to 1.0).
    pub speed_multiplier: f32,
}

impl Default for PropulsionIntent {
    fn default() -> Self {
        Self {
            direction: 0.0,
            speed_multiplier: 1.0,
        }
    }
}

impl PropulsionIntent {
    /// Create a new propulsion intent with the given direction.
    pub fn new(direction: f32) -> Self {
        Self {
            direction: direction.clamp(-1.0, 1.0),
            speed_multiplier: 1.0,
        }
    }

    /// Set the propulsion direction (-1.0 = down, 1.0 = up).
    pub fn set(&mut self, direction: f32) {
        self.direction = direction.clamp(-1.0, 1.0);
    }

    /// Set the speed multiplier.
    pub fn set_speed(&mut self, multiplier: f32) {
        self.speed_multiplier = multiplier.clamp(0.0, 1.0);
    }

    /// Clear the intent (stop propulsion).
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

    // ==================== PropulsionIntent Tests ====================

    #[test]
    fn propulsion_intent_new() {
        let intent = PropulsionIntent::new(0.5);
        assert_eq!(intent.direction, 0.5);
        assert_eq!(intent.speed_multiplier, 1.0);
    }

    #[test]
    fn propulsion_intent_clamps_direction() {
        let intent = PropulsionIntent::new(5.0);
        assert_eq!(intent.direction, 1.0);

        let intent = PropulsionIntent::new(-5.0);
        assert_eq!(intent.direction, -1.0);
    }

    #[test]
    fn propulsion_intent_set() {
        let mut intent = PropulsionIntent::default();
        intent.set(0.8);
        assert_eq!(intent.direction, 0.8);

        intent.set(-0.6);
        assert_eq!(intent.direction, -0.6);
    }

    #[test]
    fn propulsion_intent_is_active() {
        let mut intent = PropulsionIntent::default();
        assert!(!intent.is_active());

        intent.set(0.5);
        assert!(intent.is_active());

        intent.set(0.0001); // Below threshold
        assert!(!intent.is_active());
    }

    #[test]
    fn propulsion_intent_clear() {
        let mut intent = PropulsionIntent::new(1.0);
        assert!(intent.is_active());

        intent.clear();
        assert!(!intent.is_active());
        assert_eq!(intent.direction, 0.0);
    }

    #[test]
    fn propulsion_intent_effective() {
        let mut intent = PropulsionIntent::new(-1.0);
        intent.set_speed(0.5);
        assert_eq!(intent.effective(), -0.5);
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
