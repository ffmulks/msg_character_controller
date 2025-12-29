//! Respawn helper for examples.
//!
//! Provides a reusable function for respawning the player at a given position,
//! resetting all physics and controller state.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;

/// Respawns the player at the given position, resetting all physics and controller state.
///
/// This function:
/// - Sets the transform to the given spawn position (with z=1.0) and resets rotation
/// - Resets linear and angular velocity to zero
/// - Clears external impulse and force
/// - Resets the character controller (preserving gravity)
/// - Clears movement intent (including any pending jump request)
///
/// # Arguments
///
/// * `spawn_pos` - The 2D position to respawn at (will be extended to z=1.0)
/// * `transform` - The entity's transform component
/// * `velocity` - The entity's velocity component
/// * `external_impulse` - The entity's external impulse component
/// * `external_force` - The entity's external force component
/// * `controller` - The entity's character controller
/// * `movement_intent` - The entity's movement intent component
pub fn respawn_player(
    spawn_pos: Vec2,
    transform: &mut Transform,
    velocity: &mut Velocity,
    external_impulse: &mut ExternalImpulse,
    external_force: &mut ExternalForce,
    controller: &mut CharacterController,
    movement_intent: &mut MovementIntent,
) {
    // Reset position to spawn point
    transform.translation = spawn_pos.extend(1.0);
    transform.rotation = Quat::IDENTITY;

    // Reset velocity
    velocity.linvel = Vec2::ZERO;
    velocity.angvel = 0.0;

    // Reset external impulse and force
    external_impulse.impulse = Vec2::ZERO;
    external_impulse.torque_impulse = 0.0;
    external_force.force = Vec2::ZERO;
    external_force.torque = 0.0;

    // Reset controller state (keep gravity)
    let gravity = controller.gravity;
    *controller = CharacterController::with_gravity(gravity);

    // Reset movement intent and jump request
    movement_intent.clear();
    movement_intent.clear_jump_request();
}
