//! Movement intent components.
//!
//! Intents represent the desired movement direction from player input or AI.
//! The controller systems read these intents and apply appropriate physics.

use bevy::prelude::*;

/// Unified movement intent for walking and flying.
///
/// This component combines horizontal walking and vertical propulsion into
/// a single intent. The controller systems use this to apply appropriate
/// movement physics based on character state.
///
/// # Example
///
/// ```rust
/// use msg_character_controller::prelude::*;
///
/// // Create intent moving right
/// let mut intent = MovementIntent::new();
/// intent.set_walk(1.0);
/// assert!(intent.is_walking());
///
/// // Add upward propulsion
/// intent.set_fly(1.0);
/// assert!(intent.is_flying_up());
///
/// // Clear everything
/// intent.clear();
/// assert!(!intent.is_walking());
/// assert!(!intent.is_flying());
/// ```
#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
pub struct MovementIntent {
    /// Horizontal movement intent (-1.0 = left, 1.0 = right).
    pub walk: f32,
    /// Vertical propulsion intent (-1.0 = down, 1.0 = up).
    pub fly: f32,
    /// Horizontal propulsion intent (-1.0 = left, 1.0 = right).
    /// Separate from walk for flying propulsion using different controls.
    pub fly_horizontal: f32,
    /// Speed multiplier for walking (0.0 to 1.0).
    pub walk_speed: f32,
    /// Speed multiplier for flying (0.0 to 1.0).
    pub fly_speed: f32,
    /// Pending jump request, if any.
    ///
    /// Created automatically when `jump_pressed` transitions from false to true.
    /// The controller consumes this by calling `take_jump_request()`.
    pub jump_request: Option<JumpRequest>,
    /// Whether the jump action is currently active (true = wanting to jump).
    ///
    /// Set this to `true` when you want the character to jump, `false` otherwise.
    /// This is just a boolean state - you handle input detection in your code,
    /// and the controller handles the jump logic.
    ///
    /// The controller will:
    /// - Detect when this changes from `false` to `true` and create a buffered jump request
    /// - Use the held state for fall gravity calculations (shorter hops when released early)
    ///
    /// # Example
    /// ```rust,ignore
    /// // Your code handles input, we just receive a bool:
    /// intent.set_jump_pressed(keyboard.pressed(KeyCode::Space));
    /// // Or from gamepad, touch, AI, etc. - any source of a boolean
    /// intent.set_jump_pressed(gamepad.pressed(GamepadButton::South));
    /// ```
    pub jump_pressed: bool,
    /// Previous frame's jump_pressed state (for edge detection).
    /// This is managed internally by the controller.
    pub(crate) jump_pressed_prev: bool,
}

impl Default for MovementIntent {
    fn default() -> Self {
        Self {
            walk: 0.0,
            fly: 0.0,
            fly_horizontal: 0.0,
            walk_speed: 1.0,
            fly_speed: 1.0,
            jump_request: None,
            jump_pressed: false,
            jump_pressed_prev: false,
        }
    }
}

impl MovementIntent {
    /// Create a new empty movement intent.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the walking direction (-1.0 = left, 1.0 = right).
    pub fn set_walk(&mut self, direction: f32) {
        self.walk = direction.clamp(-1.0, 1.0);
    }

    /// Set the flying direction (-1.0 = down, 1.0 = up).
    pub fn set_fly(&mut self, direction: f32) {
        self.fly = direction.clamp(-1.0, 1.0);
    }

    /// Set the horizontal flying direction (-1.0 = left, 1.0 = right).
    pub fn set_fly_horizontal(&mut self, direction: f32) {
        self.fly_horizontal = direction.clamp(-1.0, 1.0);
    }

    /// Set the walk speed multiplier (0.0 to 1.0).
    pub fn set_walk_speed(&mut self, multiplier: f32) {
        self.walk_speed = multiplier.clamp(0.0, 1.0);
    }

    /// Set the fly speed multiplier (0.0 to 1.0).
    pub fn set_fly_speed(&mut self, multiplier: f32) {
        self.fly_speed = multiplier.clamp(0.0, 1.0);
    }

    /// Clear all movement intents.
    pub fn clear(&mut self) {
        self.walk = 0.0;
        self.fly = 0.0;
        self.fly_horizontal = 0.0;
    }

    /// Clear only walking intent.
    pub fn clear_walk(&mut self) {
        self.walk = 0.0;
    }

    /// Clear only flying intent.
    pub fn clear_fly(&mut self) {
        self.fly = 0.0;
    }

    /// Clear only horizontal flying intent.
    pub fn clear_fly_horizontal(&mut self) {
        self.fly_horizontal = 0.0;
    }

    /// Check if there is active walking input.
    pub fn is_walking(&self) -> bool {
        self.walk.abs() > 0.001
    }

    /// Check if there is active flying input.
    pub fn is_flying(&self) -> bool {
        self.fly.abs() > 0.001
    }

    /// Check if flying upward.
    pub fn is_flying_up(&self) -> bool {
        self.fly > 0.001
    }

    /// Check if flying downward.
    pub fn is_flying_down(&self) -> bool {
        self.fly < -0.001
    }

    /// Check if there is active horizontal flying input.
    pub fn is_flying_horizontal(&self) -> bool {
        self.fly_horizontal.abs() > 0.001
    }

    /// Check if flying left horizontally.
    pub fn is_flying_left(&self) -> bool {
        self.fly_horizontal < -0.001
    }

    /// Check if flying right horizontally.
    pub fn is_flying_right(&self) -> bool {
        self.fly_horizontal > 0.001
    }

    /// Get the effective walking direction with speed multiplier applied.
    pub fn effective_walk(&self) -> f32 {
        self.walk * self.walk_speed
    }

    /// Get the effective flying direction with speed multiplier applied.
    pub fn effective_fly(&self) -> f32 {
        self.fly * self.fly_speed
    }

    /// Get the effective horizontal flying direction with speed multiplier applied.
    pub fn effective_fly_horizontal(&self) -> f32 {
        self.fly_horizontal * self.fly_speed
    }

    /// Request a jump with the given buffer duration.
    ///
    /// **Note**: Prefer using `set_jump_pressed()` instead, which handles
    /// edge detection and timer creation automatically.
    ///
    /// Creates a jump request with a timer. The request will be consumed
    /// by apply_jump if conditions allow before the timer expires.
    pub(crate) fn request_jump(&mut self, buffer_time: f32) {
        self.jump_request = Some(JumpRequest::new(buffer_time));
    }

    /// Take and consume the pending jump request, if any.
    ///
    /// Returns the jump request if one was pending, removing it from this intent.
    pub fn take_jump_request(&mut self) -> Option<JumpRequest> {
        self.jump_request.take()
    }

    /// Check if there's a pending jump request.
    pub fn has_jump_request(&self) -> bool {
        self.jump_request.is_some()
    }

    /// Clear the pending jump request without consuming it.
    pub fn clear_jump_request(&mut self) {
        self.jump_request = None;
    }

    /// Set the jump state.
    ///
    /// Pass `true` when the player/AI wants to jump, `false` otherwise.
    /// Call this every frame with the current state. The controller will:
    /// - Detect when this changes from `false` to `true` (rising edge)
    /// - Automatically create a buffered jump request using `config.jump_buffer_time`
    /// - Track the held state for fall gravity (shorter hops when released early)
    ///
    /// You handle input detection, we handle jump logic. This works with any
    /// input source: keyboard, gamepad, touch, AI, network, etc.
    ///
    /// # Example
    /// ```rust,ignore
    /// // From keyboard:
    /// intent.set_jump_pressed(keyboard.pressed(KeyCode::Space));
    ///
    /// // From gamepad:
    /// intent.set_jump_pressed(gamepad.pressed(GamepadButton::South));
    ///
    /// // From AI:
    /// intent.set_jump_pressed(ai.wants_to_jump());
    /// ```
    pub fn set_jump_pressed(&mut self, pressed: bool) {
        self.jump_pressed = pressed;
    }

    /// Check if jump is currently active.
    ///
    /// Returns the current boolean state set via `set_jump_pressed()`.
    pub fn is_jump_pressed(&self) -> bool {
        self.jump_pressed
    }
}

/// Jump request stored in MovementIntent.
///
/// This represents a pending jump request with a timer for buffering.
/// The timer counts down from the buffer duration, and the request
/// expires when the timer finishes. The controller consumes the request
/// by taking the Option from MovementIntent.
#[derive(Reflect, Debug, Clone, Default)]
pub struct JumpRequest {
    /// Timer for jump buffering. When finished, the request expires.
    #[reflect(ignore)]
    pub buffer_timer: Timer,
}

impl JumpRequest {
    /// Create a new jump request with the given buffer duration.
    pub fn new(buffer_time: f32) -> Self {
        Self {
            buffer_timer: Timer::from_seconds(buffer_time, TimerMode::Once),
        }
    }

    /// Tick the buffer timer. Call this once per frame.
    pub fn tick(&mut self, delta: std::time::Duration) {
        self.buffer_timer.tick(delta);
    }

    /// Check if the request is still valid (timer hasn't finished).
    pub fn is_valid(&self) -> bool {
        !self.buffer_timer.finished()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== MovementIntent Tests ====================

    #[test]
    fn movement_intent_new() {
        let intent = MovementIntent::new();
        assert_eq!(intent.walk, 0.0);
        assert_eq!(intent.fly, 0.0);
        assert_eq!(intent.fly_horizontal, 0.0);
        assert_eq!(intent.walk_speed, 1.0);
        assert_eq!(intent.fly_speed, 1.0);
        assert!(intent.jump_request.is_none());
    }

    #[test]
    fn movement_intent_set_walk() {
        let mut intent = MovementIntent::new();
        intent.set_walk(0.5);
        assert_eq!(intent.walk, 0.5);

        // Clamps to valid range
        intent.set_walk(5.0);
        assert_eq!(intent.walk, 1.0);

        intent.set_walk(-5.0);
        assert_eq!(intent.walk, -1.0);
    }

    #[test]
    fn movement_intent_set_fly() {
        let mut intent = MovementIntent::new();
        intent.set_fly(0.8);
        assert_eq!(intent.fly, 0.8);

        intent.set_fly(-0.6);
        assert_eq!(intent.fly, -0.6);
    }

    #[test]
    fn movement_intent_speed_multipliers() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_walk_speed(0.5);
        assert_eq!(intent.effective_walk(), 0.5);

        intent.set_fly(-1.0);
        intent.set_fly_speed(0.5);
        assert_eq!(intent.effective_fly(), -0.5);

        // Clamps to valid range
        intent.set_walk_speed(2.0);
        assert_eq!(intent.walk_speed, 1.0);

        intent.set_fly_speed(-1.0);
        assert_eq!(intent.fly_speed, 0.0);
    }

    #[test]
    fn movement_intent_is_walking() {
        let mut intent = MovementIntent::new();
        assert!(!intent.is_walking());

        intent.set_walk(0.5);
        assert!(intent.is_walking());

        intent.set_walk(0.0001); // Below threshold
        assert!(!intent.is_walking());
    }

    #[test]
    fn movement_intent_is_flying() {
        let mut intent = MovementIntent::new();
        assert!(!intent.is_flying());
        assert!(!intent.is_flying_up());
        assert!(!intent.is_flying_down());

        intent.set_fly(0.5);
        assert!(intent.is_flying());
        assert!(intent.is_flying_up());
        assert!(!intent.is_flying_down());

        intent.set_fly(-0.5);
        assert!(intent.is_flying());
        assert!(!intent.is_flying_up());
        assert!(intent.is_flying_down());
    }

    #[test]
    fn movement_intent_set_fly_horizontal() {
        let mut intent = MovementIntent::new();
        intent.set_fly_horizontal(0.5);
        assert_eq!(intent.fly_horizontal, 0.5);

        intent.set_fly_horizontal(-0.8);
        assert_eq!(intent.fly_horizontal, -0.8);

        // Clamps to valid range
        intent.set_fly_horizontal(2.0);
        assert_eq!(intent.fly_horizontal, 1.0);

        intent.set_fly_horizontal(-2.0);
        assert_eq!(intent.fly_horizontal, -1.0);
    }

    #[test]
    fn movement_intent_is_flying_horizontal() {
        let mut intent = MovementIntent::new();
        assert!(!intent.is_flying_horizontal());
        assert!(!intent.is_flying_left());
        assert!(!intent.is_flying_right());

        intent.set_fly_horizontal(0.5);
        assert!(intent.is_flying_horizontal());
        assert!(intent.is_flying_right());
        assert!(!intent.is_flying_left());

        intent.set_fly_horizontal(-0.5);
        assert!(intent.is_flying_horizontal());
        assert!(intent.is_flying_left());
        assert!(!intent.is_flying_right());
    }

    #[test]
    fn movement_intent_clear() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_fly(1.0);
        intent.set_fly_horizontal(1.0);

        intent.clear();
        assert!(!intent.is_walking());
        assert!(!intent.is_flying());
        assert!(!intent.is_flying_horizontal());
        assert_eq!(intent.walk, 0.0);
        assert_eq!(intent.fly, 0.0);
        assert_eq!(intent.fly_horizontal, 0.0);
    }

    #[test]
    fn movement_intent_clear_fly_horizontal() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_fly(1.0);
        intent.set_fly_horizontal(1.0);

        intent.clear_fly_horizontal();
        assert!(intent.is_walking());
        assert!(intent.is_flying());
        assert!(!intent.is_flying_horizontal());
    }

    #[test]
    fn movement_intent_clear_walk() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_fly(1.0);

        intent.clear_walk();
        assert!(!intent.is_walking());
        assert!(intent.is_flying());
    }

    #[test]
    fn movement_intent_clear_fly() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_fly(1.0);

        intent.clear_fly();
        assert!(intent.is_walking());
        assert!(!intent.is_flying());
    }

    // ==================== JumpRequest Tests ====================

    #[test]
    fn jump_request_new() {
        let request = JumpRequest::new(0.1);
        // New request should be valid (timer not finished)
        assert!(request.is_valid());
    }

    #[test]
    fn jump_request_is_valid_after_tick() {
        use std::time::Duration;

        let mut request = JumpRequest::new(0.1); // 100ms buffer

        // After 50ms, should still be valid
        request.tick(Duration::from_millis(50));
        assert!(request.is_valid());

        // After another 60ms (total 110ms), should be expired
        request.tick(Duration::from_millis(60));
        assert!(!request.is_valid());
    }

    #[test]
    fn jump_request_expires_at_buffer_time() {
        use std::time::Duration;

        let mut request = JumpRequest::new(0.1); // 100ms buffer

        // Tick just before buffer time - should still be valid
        request.tick(Duration::from_millis(99));
        assert!(request.is_valid());

        // Tick past buffer time - should be expired
        request.tick(Duration::from_millis(2));
        assert!(!request.is_valid());
    }

    // ==================== MovementIntent Jump Tests ====================

    #[test]
    fn movement_intent_request_jump() {
        let mut intent = MovementIntent::new();
        assert!(!intent.has_jump_request());

        intent.request_jump(0.1);
        assert!(intent.has_jump_request());
        assert!(intent.jump_request.as_ref().unwrap().is_valid());
    }

    #[test]
    fn movement_intent_take_jump_request() {
        let mut intent = MovementIntent::new();
        intent.request_jump(0.1);

        let request = intent.take_jump_request();
        assert!(request.is_some());
        assert!(request.unwrap().is_valid());

        // Should be consumed now
        assert!(!intent.has_jump_request());
        assert!(intent.take_jump_request().is_none());
    }

    #[test]
    fn movement_intent_clear_jump_request() {
        let mut intent = MovementIntent::new();
        intent.request_jump(0.1);
        assert!(intent.has_jump_request());

        intent.clear_jump_request();
        assert!(!intent.has_jump_request());
    }

    #[test]
    fn movement_intent_request_jump_always_overwrites() {
        use std::time::Duration;

        let mut intent = MovementIntent::new();
        intent.request_jump(0.1); // 100ms buffer

        // Tick the first request partially
        if let Some(ref mut jump) = intent.jump_request {
            jump.tick(Duration::from_millis(50));
        }

        // Request again with fresh buffer
        intent.request_jump(0.2); // 200ms buffer

        // New request should be valid and have fresh timer
        assert!(intent.jump_request.as_ref().unwrap().is_valid());
    }

    #[test]
    fn movement_intent_set_jump_pressed() {
        let mut intent = MovementIntent::new();
        assert!(!intent.is_jump_pressed());

        intent.set_jump_pressed(true);
        assert!(intent.is_jump_pressed());

        intent.set_jump_pressed(false);
        assert!(!intent.is_jump_pressed());
    }

    #[test]
    fn movement_intent_jump_pressed_default() {
        let intent = MovementIntent::new();
        assert!(!intent.jump_pressed);
        assert!(!intent.jump_pressed_prev);
    }
}
