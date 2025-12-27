//! Core controller systems.
//!
//! These systems implement the floating character controller behavior.
//! They are generic over the physics backend to allow different physics
//! engines to be used.

use std::f32::consts;

use bevy::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, CharacterOrientation, ControllerConfig};
use crate::intent::{FlyIntent, JumpRequest, WalkIntent};
use crate::state::{Airborne, Grounded, TouchingCeiling, TouchingWall};
use crate::{GravityMode, GravityModeResource};

/// Apply the floating spring force to maintain float height.
///
/// This system reads ground detection data from the CharacterController
/// and applies spring forces to maintain the configured float height.
pub fn apply_floating_spring<B: CharacterPhysicsBackend>(world: &mut World) {
    // Collect entities that need floating spring
    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation, CharacterController)> =
        world
            .query::<(
                Entity,
                &ControllerConfig,
                Option<&CharacterOrientation>,
                &CharacterController,
            )>()
            .iter(world)
            .filter(|(_, _, _, controller)| controller.is_walking())
            .map(|(e, config, orientation, controller)| {
                (
                    e,
                    *config,
                    orientation.copied().unwrap_or_default(),
                    controller.clone(),
                )
            })
            .collect();

    for (entity, config, orientation, controller) in entities {
        if !controller.ground_detected {
            continue;
        }

        let velocity = B::get_velocity(world, entity);
        let up = orientation.up();
        let gravity = controller.gravity;

        // Calculate height error (positive = below float height, negative = above)
        let height_error = controller.height_error(config.float_height);
        let vertical_velocity = velocity.dot(up);

        // Float height is a FLOOR, not a target!
        // Only apply spring force when BELOW float height (height_error > 0)
        // Never pull the character DOWN when they're above float height (e.g., jumping)
        if height_error > 0.0 {
            // Spring force: F = k * x - c * v
            // Only apply when below float height to push character up
            let spring_force =
                config.spring_strength * height_error - config.spring_damping * vertical_velocity;

            let force = up * spring_force;
            B::apply_force(world, entity, force);
        }

        // Apply cling force when above float height but within cling distance
        // This helps the character stick to ground on bumpy terrain
        // But skip if we're moving upward fast (probably jumping)
        if height_error < 0.0
            && controller.ground_distance <= config.float_height + config.cling_distance
            && config.cling_strength > 0.0
            && vertical_velocity < 100.0
        {
            let cling_force = gravity * config.cling_strength;
            B::apply_force(world, entity, cling_force);
        }

        // Apply extra fall gravity when falling (velocity is in -UP direction)
        // This makes the character fall faster, creating a more responsive feel
        // Uses the controller's gravity field
        if vertical_velocity < 0.0 && config.extra_fall_gravity > 0.0 {
            let extra_gravity_force = gravity * config.extra_fall_gravity;
            B::apply_force(world, entity, extra_gravity_force);
        }
    }
}

/// Apply internal gravity when configured.
///
/// This system applies gravity from CharacterController.gravity when:
/// - GravityMode is Internal
/// - Character is walking
/// - Character is not grounded
pub fn apply_internal_gravity<B: CharacterPhysicsBackend>(world: &mut World) {
    // Check gravity mode
    let gravity_mode = world
        .get_resource::<GravityModeResource>()
        .map(|r| r.0)
        .unwrap_or(GravityMode::Internal);

    if gravity_mode != GravityMode::Internal {
        return;
    }

    let dt = B::get_fixed_timestep(world);

    // Collect entities needing gravity
    let entities: Vec<(Entity, CharacterController)> = world
        .query::<(Entity, &CharacterController)>()
        .iter(world)
        .filter(|(_, controller)| controller.is_walking() && !controller.is_grounded)
        .map(|(e, controller)| (e, controller.clone()))
        .collect();

    for (entity, controller) in entities {
        // Apply gravity as a velocity change (acceleration)
        let velocity = B::get_velocity(world, entity);
        let new_velocity = velocity + controller.gravity * dt;
        B::set_velocity(world, entity, new_velocity);
    }
}

/// Apply walking movement based on intent.
pub fn apply_walk_movement<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation, WalkIntent, CharacterController)> =
        world
            .query::<(
                Entity,
                &ControllerConfig,
                Option<&CharacterOrientation>,
                &WalkIntent,
                &CharacterController,
            )>()
            .iter(world)
            .filter(|(_, _, _, _, controller)| controller.is_walking())
            .map(|(e, config, orientation, intent, controller)| {
                (
                    e,
                    *config,
                    orientation.copied().unwrap_or_default(),
                    *intent,
                    controller.clone(),
                )
            })
            .collect();

    // Get fixed timestep delta, with fallback for testing scenarios
    let dt = world
        .get_resource::<Time<Fixed>>()
        .map(|t| t.delta_secs())
        .filter(|&d| d > 0.0)
        .unwrap_or(1.0 / 60.0);

    for (entity, config, orientation, intent, controller) in entities {
        let current_velocity = B::get_velocity(world, entity);
        let up = orientation.up();

        // Determine movement direction based on ground normal (tangent to slope)
        // or use the character's local right direction if not grounded
        let right = if controller.ground_detected {
            controller.ground_tangent()
        } else {
            orientation.right()
        };

        // Calculate desired velocity
        let desired_speed = intent.effective() * config.max_speed;

        // Determine acceleration based on grounded state
        let accel = if controller.is_grounded {
            config.acceleration
        } else {
            config.acceleration * config.air_control
        };

        // Get horizontal component of current velocity (along movement axis)
        let current_horizontal = current_velocity.dot(right);

        // Calculate velocity change
        let velocity_diff = desired_speed - current_horizontal;
        let max_change = accel * dt;
        let change = velocity_diff.clamp(-max_change, max_change);

        // Apply friction when no input
        let friction_multiplier = if !intent.is_active() && controller.is_grounded {
            1.0 - config.friction
        } else {
            1.0
        };

        // Calculate new horizontal velocity
        let new_horizontal = (current_horizontal + change) * friction_multiplier;

        // Preserve vertical velocity (along up direction), replace horizontal
        let vertical_velocity = current_velocity.dot(up);

        let new_velocity = right * new_horizontal + up * vertical_velocity;
        B::set_velocity(world, entity, new_velocity);
    }
}

/// Apply flying movement based on intent.
///
/// The FlyIntent is interpreted in the character's local coordinate system:
/// - intent.x moves along the character's right direction
/// - intent.y moves along the character's up direction
pub fn apply_fly_movement<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation, FlyIntent)> = world
        .query::<(
            Entity,
            &CharacterController,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            &FlyIntent,
        )>()
        .iter(world)
        .filter(|(_, controller, _, _, _)| controller.is_flying())
        .map(|(e, _, config, orientation, intent)| {
            (
                e,
                *config,
                orientation.copied().unwrap_or_default(),
                *intent,
            )
        })
        .collect();

    // Get fixed timestep delta, with fallback for testing scenarios
    let dt = world
        .get_resource::<Time<Fixed>>()
        .map(|t| t.delta_secs())
        .filter(|&d| d > 0.0)
        .unwrap_or(1.0 / 60.0);

    for (entity, config, orientation, intent) in entities {
        let current_velocity = B::get_velocity(world, entity);

        // Convert local intent to world velocity
        let local_intent = intent.effective();
        let desired_velocity = orientation.to_world(local_intent) * config.max_speed;

        // Full air control for flying
        let accel = config.acceleration;

        // Calculate velocity change
        let velocity_diff = desired_velocity - current_velocity;
        let max_change = accel * dt;

        let change = if velocity_diff.length_squared() > max_change * max_change {
            velocity_diff.normalize() * max_change
        } else {
            velocity_diff
        };

        // Apply friction when no input
        let friction_multiplier = if !intent.is_active() {
            1.0 - config.friction
        } else {
            1.0
        };

        let new_velocity = (current_velocity + change) * friction_multiplier;
        B::set_velocity(world, entity, new_velocity);
    }
}

/// Apply jump impulse when requested.
///
/// Uses the CharacterController's gravity to calculate jump counter force.
pub fn apply_jump<B: CharacterPhysicsBackend>(world: &mut World) {
    let time = world
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0);

    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation, CharacterController, bool)> =
        world
            .query::<(
                Entity,
                &ControllerConfig,
                Option<&CharacterOrientation>,
                &CharacterController,
                &JumpRequest,
            )>()
            .iter(world)
            .filter(|(_, _, _, controller, _)| controller.is_walking())
            .map(|(e, config, orientation, controller, jump)| {
                let can_jump = jump.is_valid(time, config.jump_buffer_time)
                    && (controller.is_grounded
                        || controller.time_since_grounded < config.coyote_time);
                (
                    e,
                    *config,
                    orientation.copied().unwrap_or_default(),
                    controller.clone(),
                    can_jump,
                )
            })
            .collect();

    for (entity, config, orientation, controller, can_jump) in entities {
        if !can_jump {
            continue;
        }

        // Consume the jump request
        if let Some(mut jump) = world.get_mut::<JumpRequest>(entity) {
            jump.consume();
        }

        // Calculate jump direction: use ground normal if detected, otherwise character's up
        let up = if controller.ground_detected {
            controller.ground_normal
        } else {
            orientation.up()
        };

        // Apply jump impulse
        let impulse = up * config.jump_speed;
        B::apply_impulse(world, entity, impulse);
    }
}

/// Sync state marker components based on CharacterController detection results.
pub fn sync_state_markers(
    mut commands: Commands,
    q_controllers: Query<(
        Entity,
        &CharacterController,
        Option<&CharacterOrientation>,
        Has<Grounded>,
        Has<Airborne>,
        Has<TouchingWall>,
        Has<TouchingCeiling>,
    )>,
) {
    for (entity, controller, orientation_opt, has_grounded, has_airborne, has_wall, has_ceiling) in
        &q_controllers
    {
        let orientation = orientation_opt.copied().unwrap_or_default();

        // Sync Grounded/Airborne
        if controller.is_grounded && !has_grounded {
            commands.entity(entity).insert(Grounded);
            commands.entity(entity).remove::<Airborne>();
        } else if !controller.is_grounded && has_grounded {
            commands.entity(entity).remove::<Grounded>();
            commands.entity(entity).insert(Airborne);
        } else if !controller.is_grounded && !has_airborne && !has_grounded {
            commands.entity(entity).insert(Airborne);
        }

        // Sync TouchingWall using character's local directions
        let touching_wall = controller.touching_left_wall || controller.touching_right_wall;
        if touching_wall && !has_wall {
            let (direction, normal) = if controller.touching_left_wall {
                (orientation.left(), controller.left_wall_normal)
            } else {
                (orientation.right(), controller.right_wall_normal)
            };
            commands
                .entity(entity)
                .insert(TouchingWall::new(direction, normal));
        } else if !touching_wall && has_wall {
            commands.entity(entity).remove::<TouchingWall>();
        }

        // Sync TouchingCeiling
        if controller.touching_ceiling && !has_ceiling {
            commands
                .entity(entity)
                .insert(TouchingCeiling::new(0.0, controller.ceiling_normal));
        } else if !controller.touching_ceiling && has_ceiling {
            commands.entity(entity).remove::<TouchingCeiling>();
        }
    }
}

/// Reset jump requests at the end of each frame.
pub fn reset_jump_requests(mut q: Query<&mut JumpRequest>) {
    for mut jump in &mut q {
        if jump.consumed {
            jump.reset();
        }
    }
}

/// Apply upright torque to keep characters oriented correctly.
///
/// Uses a spring-damper system with quadratic spring behavior for strong correction.
/// The torque is proportional to the square of the angle error, creating a force
/// that increases rapidly as the character tilts further from upright.
pub fn apply_upright_torque<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation)> = world
        .query::<(
            Entity,
            &CharacterController,
            &ControllerConfig,
            Option<&CharacterOrientation>,
        )>()
        .iter(world)
        .filter(|(_, _, config, _)| config.upright_torque_enabled)
        .map(|(e, _, config, orientation)| (e, *config, orientation.copied().unwrap_or_default()))
        .collect();

    for (entity, config, orientation) in entities {
        // Get current rotation and angular velocity
        let current_rotation = B::get_rotation(world, entity);
        let angular_velocity = B::get_angular_velocity(world, entity);

        // Calculate the target angle from the orientation's up direction
        // or use the config's upright_target_angle if specified
        let target_angle = config
            .upright_target_angle
            .unwrap_or_else(|| orientation.angle() - consts::FRAC_PI_2);

        // Calculate angle error, normalized to [-PI, PI]
        let mut angle_error = target_angle - current_rotation;
        while angle_error > consts::PI {
            angle_error -= consts::TAU;
        }
        while angle_error < -consts::PI {
            angle_error += consts::TAU;
        }

        // Apply quadratic spring force: F = strength * error² * sign(error)
        // This creates a stronger corrective force the further from upright
        let spring_torque =
            config.upright_torque_strength * angle_error * angle_error * angle_error.signum();

        // Apply damping to reduce oscillation
        let damping_torque = -config.upright_torque_damping * angular_velocity;

        let total_torque = spring_torque + damping_torque;
        B::apply_torque(world, entity, total_torque);
    }
}
