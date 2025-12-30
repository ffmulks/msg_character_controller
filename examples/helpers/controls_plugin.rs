//! Controls Plugin for examples.
//!
//! Provides a reusable plugin for keyboard input handling and camera following
//! in character controller examples.
//!
//! ## Controls
//! - **A/D**: Walk horizontally (with ground friction)
//! - **W**: Jump
//! - **Arrow Keys**: Flying propulsion in all directions (up/down/left/right)
//!   - Uses separate flying configuration (fly_max_speed, vertical ratio, etc.)
//!   - Horizontal flying uses full speed regardless of grounded state

use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;
use msg_character_controller::prelude::*;

/// Marker component for the player entity.
///
/// Add this component to your player entity to enable input handling
/// and camera following from the `ControlsPlugin`.
#[derive(Component)]
pub struct Player;

/// Plugin that provides keyboard input handling and optionally camera following for examples.
///
/// # Usage
///
/// ```rust,ignore
/// use helpers::{ControlsPlugin, Player};
///
/// fn main() {
///     App::new()
///         // With default camera following
///         .add_plugins(ControlsPlugin::default())
///         // Or without camera following (for custom camera behavior)
///         .add_plugins(ControlsPlugin::input_only())
///         // ...
///         .run();
/// }
///
/// fn spawn_player(mut commands: Commands) {
///     commands.spawn((
///         Player,
///         // ... other components
///     ));
/// }
/// ```
pub struct ControlsPlugin {
    /// Whether to enable the default camera follow system.
    camera_follow: bool,
}

impl Default for ControlsPlugin {
    fn default() -> Self {
        Self::new()
    }
}

impl ControlsPlugin {
    /// Creates a new ControlsPlugin with both input handling and camera following.
    pub fn new() -> Self {
        Self {
            camera_follow: true,
        }
    }

    /// Creates a ControlsPlugin with only input handling (no camera follow).
    ///
    /// Use this when you want to implement custom camera behavior.
    pub fn input_only() -> Self {
        Self {
            camera_follow: false,
        }
    }

    /// Enables or disables camera following.
    pub fn with_camera_follow(mut self, enabled: bool) -> Self {
        self.camera_follow = enabled;
        self
    }
}

impl Plugin for ControlsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, handle_input);

        if self.camera_follow {
            app.add_systems(Update, camera_follow);
        }
    }
}

/// Handles keyboard input for movement and jumping.
///
/// This system reads keyboard input and updates `MovementIntent` on entities
/// with the `Player` marker. Jump input is handled by simply forwarding the
/// button state - the system automatically detects rising edges and creates
/// jump requests with the configured buffer time.
///
/// Controls:
/// - **A/D**: Walk horizontally (uses ground friction when grounded)
/// - **W**: Jump (press to jump, hold for higher jumps)
/// - **Arrow Keys**: Flying propulsion (up/down/left/right)
///   - Uses fly_max_speed and fly_vertical_speed_ratio configs
///   - Horizontal flying uses full speed (no air control reduction)
///   - When grounded, horizontal flying uses walking friction
///
/// Input is disabled when egui wants keyboard focus (e.g., when typing in a text field).
fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    egui_wants_input: Res<EguiWantsInput>,
    mut query: Query<&mut MovementIntent, With<Player>>,
) {
    // Skip input handling if egui wants keyboard input
    if egui_wants_input.wants_any_keyboard_input() {
        // Clear any ongoing movement when egui takes focus
        for mut movement in &mut query {
            movement.clear();
            movement.set_jump_pressed(false);
        }
        return;
    }

    for mut movement in &mut query {
        // Walking input (A/D only)
        let mut walk = 0.0;
        if keyboard.pressed(KeyCode::KeyA) {
            walk -= 1.0;
        }
        if keyboard.pressed(KeyCode::KeyD) {
            walk += 1.0;
        }
        movement.set_walk(walk);

        // Flying propulsion - Arrow keys for all directions
        // Vertical propulsion (Arrow Up/Down)
        let mut fly_vertical = 0.0;
        if keyboard.pressed(KeyCode::ArrowUp) {
            fly_vertical += 1.0;
        }
        if keyboard.pressed(KeyCode::ArrowDown) {
            fly_vertical -= 1.0;
        }
        movement.set_fly(fly_vertical);

        // Horizontal flying propulsion (Arrow Left/Right)
        let mut fly_horizontal = 0.0;
        if keyboard.pressed(KeyCode::ArrowLeft) {
            fly_horizontal -= 1.0;
        }
        if keyboard.pressed(KeyCode::ArrowRight) {
            fly_horizontal += 1.0;
        }
        movement.set_fly_horizontal(fly_horizontal);

        // Jump on W only - pass the current button state as a bool
        // The controller handles edge detection, buffering, and all jump logic
        let wants_to_jump = keyboard.pressed(KeyCode::KeyW);
        movement.set_jump_pressed(wants_to_jump);
    }
}

/// Smoothly follows the player with the camera.
///
/// Uses linear interpolation to create a smooth camera follow effect.
/// This is the default camera behavior; for custom behavior (e.g., planetary
/// examples), use `ControlsPlugin::input_only()` and implement your own.
fn camera_follow(
    player_query: Query<&Transform, (With<Player>, Without<Camera2d>)>,
    mut camera_query: Query<&mut Transform, With<Camera2d>>,
) {
    let Ok(player_transform) = player_query.single() else {
        return;
    };

    let Ok(mut camera_transform) = camera_query.single_mut() else {
        return;
    };

    // Smooth camera follow
    let target = player_transform.translation.xy();
    let current = camera_transform.translation.xy();
    let smoothed = current.lerp(target, 0.1);
    camera_transform.translation.x = smoothed.x;
    camera_transform.translation.y = smoothed.y;
}
