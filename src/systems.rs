//! Core controller systems.
//!
//! These systems implement the floating character controller behavior.
//! They are generic over the physics backend to allow different physics
//! engines to be used.

use std::f32::consts;

use bevy::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, CharacterOrientation, ControllerConfig};
use crate::intent::{JumpRequest, PropulsionIntent, WalkIntent};
use crate::state::{Airborne, Grounded, TouchingCeiling, TouchingWall};
use crate::{GravityMode, GravityModeResource};

/// Apply the floating spring force to maintain float height.
///
/// This system uses a proper PD (proportional-derivative) controller to maintain
/// the character at float_height above ground. The spring applies force in BOTH
/// directions - pushing up when below target, pushing down when above target.
/// This creates stable hovering behavior.
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
        // Only apply spring when ground is detected
        if !controller.ground_detected {
            continue;
        }

        let velocity = B::get_velocity(world, entity);
        let up = orientation.up();

        // Calculate height error (positive = below float height, negative = above)
        let height_error = config.float_height - controller.ground_distance;
        let vertical_velocity = velocity.dot(up);

        // Apply spring force using PD control: F = k * error - d * velocity
        // This works in BOTH directions:
        // - When below target (error > 0): push UP
        // - When above target (error < 0): push DOWN (soft landing)
        //
        // The damping term reduces oscillation and provides smooth settling.
        let spring_force =
            config.spring_strength * height_error - config.spring_damping * vertical_velocity;

        // Clamp force to prevent extreme values
        let max_force = config.spring_strength * config.float_height * 2.0;
        let clamped_force = spring_force.clamp(-max_force, max_force);

        let force = up * clamped_force;
        B::apply_force(world, entity, force);
    }
}

/// Apply internal gravity when configured.
///
/// This system applies gravity from CharacterController.gravity when:
/// - GravityMode is Internal
/// - Character is NOT grounded (airborne)
///
/// Gravity is applied as a velocity change (acceleration) per physics frame.
/// When grounded, the floating spring maintains height instead.
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
        .filter(|(_, controller)| !controller.is_grounded)
        .map(|(e, controller)| (e, controller.clone()))
        .collect();

    for (entity, controller) in entities {
        // Apply gravity as acceleration (velocity change)
        // This is the standard physics approach: v += g * dt
        let velocity = B::get_velocity(world, entity);
        let new_velocity = velocity + controller.gravity * dt;
        B::set_velocity(world, entity, new_velocity);
    }
}

/// Apply horizontal walking movement based on intent.
///
/// Movement is applied along the ground tangent when grounded, or along the
/// character's horizontal axis when airborne. Vertical velocity is always
/// preserved correctly using orthogonal decomposition.
pub fn apply_walk_movement<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterOrientation,
        WalkIntent,
        CharacterController,
    )> = world
        .query::<(
            Entity,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            &WalkIntent,
            &CharacterController,
        )>()
        .iter(world)
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

        // Get orthogonal axes for velocity decomposition
        // CRITICAL: Both axes MUST be orthogonal to avoid velocity leakage
        let (move_axis, vertical_axis) = if controller.ground_detected && controller.is_grounded {
            // On ground: use ground tangent for movement, ground normal for vertical
            // These are guaranteed orthogonal
            let tangent = controller.ground_tangent();
            let normal = controller.ground_normal;
            (tangent, normal)
        } else {
            // In air: use character's local axes
            (orientation.right(), orientation.up())
        };

        // Decompose velocity into orthogonal components
        let current_horizontal = current_velocity.dot(move_axis);
        let current_vertical = current_velocity.dot(vertical_axis);

        // Calculate desired horizontal velocity
        let desired_speed = intent.effective() * config.max_speed;

        // Determine acceleration based on grounded state
        let accel = if controller.is_grounded {
            config.acceleration
        } else {
            config.acceleration * config.air_control
        };

        // Calculate velocity change
        let velocity_diff = desired_speed - current_horizontal;
        let max_change = accel * dt;
        let change = velocity_diff.clamp(-max_change, max_change);

        // Apply friction when no input and grounded
        let friction_multiplier = if !intent.is_active() && controller.is_grounded {
            1.0 - config.friction
        } else {
            1.0
        };

        // Calculate new horizontal velocity
        let new_horizontal = (current_horizontal + change) * friction_multiplier;

        // Reconstruct velocity using ORTHOGONAL axes
        // This preserves vertical velocity exactly
        let new_velocity = move_axis * new_horizontal + vertical_axis * current_vertical;
        B::set_velocity(world, entity, new_velocity);
    }
}

/// Apply vertical propulsion based on intent.
///
/// This system provides vertical thrust (jetpack, thrusters, etc.):
/// - Positive intent thrusts upward
/// - Negative intent thrusts downward
/// - Upward thrust is automatically boosted by gravity magnitude to help counteract it
///
/// This is independent of horizontal walking movement and jump impulses.
pub fn apply_propulsion<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterOrientation,
        PropulsionIntent,
        CharacterController,
    )> = world
        .query::<(
            Entity,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            &PropulsionIntent,
            &CharacterController,
        )>()
        .iter(world)
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
        if !intent.is_active() {
            continue;
        }

        let current_velocity = B::get_velocity(world, entity);
        let up = orientation.up();

        // Get current vertical velocity
        let current_vertical = current_velocity.dot(up);

        // Desired vertical speed based on intent
        let desired_vertical = intent.effective() * config.max_speed;

        // Base acceleration
        let mut accel = config.acceleration;

        // Boost upward propulsion by gravity magnitude to help counteract it
        if intent.direction > 0.0 {
            accel += controller.gravity.length();
        }

        // Calculate velocity change
        let velocity_diff = desired_vertical - current_vertical;
        let max_change = accel * dt;
        let change = velocity_diff.clamp(-max_change, max_change);

        // Apply the vertical velocity change while preserving horizontal
        let right = orientation.right();
        let horizontal_velocity = current_velocity.dot(right);
        let new_vertical = current_vertical + change;

        let new_velocity = right * horizontal_velocity + up * new_vertical;
        B::set_velocity(world, entity, new_velocity);
    }
}

/// Apply jump impulse when requested.
///
/// Jumping requires being grounded (or within coyote time).
/// Uses the CharacterController's gravity to calculate jump counter force.
pub fn apply_jump<B: CharacterPhysicsBackend>(world: &mut World) {
    let time = world
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0);

    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterOrientation,
        CharacterController,
        bool,
    )> = world
        .query::<(
            Entity,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            &CharacterController,
            &JumpRequest,
        )>()
        .iter(world)
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

        // Calculate jump direction: use ground normal if grounded, otherwise character's up
        let up = if controller.is_grounded {
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
