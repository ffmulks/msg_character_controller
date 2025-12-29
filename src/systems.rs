//! Core controller systems.
//!
//! These systems implement the floating character controller behavior.
//! They are generic over the physics backend to allow different physics
//! engines to be used.

use std::f32::consts;

use bevy::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, CharacterOrientation, ControllerConfig};
use crate::intent::MovementIntent;

/// Apply the floating spring force to maintain riding height.
///
/// Simple spring-damper: F = k * displacement - c * velocity
/// - displacement = target_height - current_height (positive = below target)
/// - velocity = vertical velocity (positive = moving up)
pub fn apply_floating_spring<B: CharacterPhysicsBackend>(world: &mut World) {
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
        let Some(ref floor) = controller.floor else {
            continue;
        };

        let up = orientation.up();
        let velocity = B::get_velocity(world, entity);
        let vertical_velocity = velocity.dot(up);

        // Target height and current height
        let target_height = controller.riding_height(&config);
        let current_height = floor.distance;

        // Only apply spring within active range
        let max_range = target_height + config.ground_tolerance;
        let min_range = controller.capsule_half_height();
        if current_height > max_range || current_height < min_range {
            continue;
        }

        // Spring-damper formula: F = k * x - c * v
        // x = displacement from target (positive = below target, needs push up)
        // v = vertical velocity (positive = moving up, damp it)
        let displacement = target_height - current_height;
        let spring_force = config.spring_strength * displacement - config.spring_damping * vertical_velocity;

        // Clamp spring force to prevent overflow.
        // Maximum force is based on counteracting gravity plus reasonable acceleration.
        // This prevents the damping term from creating runaway forces when
        // entering the spring zone at high velocity.
        let gravity_magnitude = controller.gravity.length();
        let max_spring_force = gravity_magnitude * 3.0 + config.spring_strength * config.ground_tolerance;
        let clamped_spring_force = spring_force.clamp(-max_spring_force, max_spring_force);

        // Apply force along up direction
        let force = up * clamped_spring_force;
        B::apply_force(world, entity, force);
    }
}

/// Apply gravity as a force.
///
/// Gravity is applied from CharacterController.gravity as a force when the
/// character is not grounded. The gravity is applied as an impulse each
/// physics frame to produce the equivalent acceleration.
///
/// Note: Gravity is always applied internally by this system. To change the
/// gravity affecting a character, modify CharacterController::gravity directly.
pub fn apply_gravity<B: CharacterPhysicsBackend>(world: &mut World) {
    let dt = B::get_fixed_timestep(world);

    // Collect entities needing gravity
    let entities: Vec<(Entity, CharacterController, ControllerConfig)> = world
        .query::<(Entity, &CharacterController, &ControllerConfig)>()
        .iter(world)
        .filter(|(_, controller, config)| !controller.is_grounded(config))
        .map(|(e, controller, config)| (e, controller.clone(), *config))
        .collect();

    for (entity, controller, _config) in entities {
        // Apply gravity as an impulse: I = m * g * dt
        // This produces velocity change: dv = I / m = g * dt
        // Using impulse ensures gravity is integrated correctly with the physics step
        let actual_mass = B::get_mass(world, entity);
        let gravity_impulse = controller.gravity * actual_mass * dt;
        B::apply_impulse(world, entity, gravity_impulse);
    }
}

/// Apply movement (walking and flying) based on intent.
///
/// This unified system handles both horizontal walking and vertical propulsion.
/// Flying downwards is disabled while grounded.
pub fn apply_movement<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterOrientation,
        MovementIntent,
        CharacterController,
    )> = world
        .query::<(
            Entity,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            &MovementIntent,
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

    // Get fixed timestep delta
    let dt = world
        .get_resource::<Time<Fixed>>()
        .map(|t| t.delta_secs())
        .filter(|&d| d > 0.0)
        .unwrap_or(1.0 / 60.0);

    for (entity, config, orientation, intent, controller) in entities {
        let current_velocity = B::get_velocity(world, entity);
        let up = orientation.up();
        let right = orientation.right();

        let is_grounded = controller.is_grounded(&config);

        // === WALKING ===
        let desired_walk_speed = intent.effective_walk() * config.max_speed;
        let walk_accel = if is_grounded {
            config.acceleration
        } else {
            config.acceleration * config.air_control
        };

        let new_velocity = if is_grounded && controller.ground_detected() {
            // GROUNDED: Move along slope surface
            let slope_tangent = controller.ground_tangent();
            let slope_normal = controller.ground_normal();

            let current_slope_speed = current_velocity.dot(slope_tangent);
            let velocity_diff = desired_walk_speed - current_slope_speed;
            let max_change = walk_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            let friction_multiplier = if !intent.is_walking() {
                1.0 - config.friction
            } else {
                1.0
            };

            let new_slope_speed = (current_slope_speed + change) * friction_multiplier;

            let normal_velocity = current_velocity.dot(slope_normal);
            let preserved_normal = if normal_velocity < 0.0 {
                normal_velocity * 0.5
            } else {
                0.0
            };

            slope_tangent * new_slope_speed + slope_normal * preserved_normal
        } else {
            // AIRBORNE: Use world-space coordinates
            let current_horizontal = current_velocity.dot(right);
            let velocity_diff = desired_walk_speed - current_horizontal;
            let max_change = walk_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);
            let new_horizontal = current_horizontal + change;

            let vertical_velocity = current_velocity.dot(up);
            right * new_horizontal + up * vertical_velocity
        };

        // === FLYING ===
        // Flying downwards is disabled while grounded
        let fly_direction = intent.fly;
        let should_apply_fly = intent.is_flying() && !(is_grounded && intent.is_flying_down());

        let final_velocity = if should_apply_fly {
            let current_vertical = new_velocity.dot(up);
            let desired_vertical = intent.effective_fly() * config.max_speed;

            let mut fly_accel = config.acceleration;
            // Boost upward propulsion by gravity magnitude
            if fly_direction > 0.0 {
                fly_accel += controller.gravity.length();
            }

            let velocity_diff = desired_vertical - current_vertical;
            let max_change = fly_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);
            let new_vertical = current_vertical + change;

            let horizontal_velocity = new_velocity.dot(right);
            right * horizontal_velocity + up * new_vertical
        } else {
            new_velocity
        };

        B::set_velocity(world, entity, final_velocity);
    }
}

/// Apply jump impulse when requested.
///
/// Jumping requires being grounded (or within coyote time).
/// Jump requests are consumed directly from MovementIntent.
pub fn apply_jump<B: CharacterPhysicsBackend>(world: &mut World) {
    let time = world
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0);

    // Collect entities with pending jump requests that can jump
    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation, CharacterController)> =
        world
            .query::<(
                Entity,
                &ControllerConfig,
                Option<&CharacterOrientation>,
                &CharacterController,
                &MovementIntent,
            )>()
            .iter(world)
            .filter_map(|(e, config, orientation, controller, intent)| {
                // Check if there's a valid jump request
                let jump = intent.jump_request.as_ref()?;
                let is_within_buffer = jump.is_within_buffer(time, config.jump_buffer_time);
                let can_jump_now = controller.is_grounded(config)
                    || controller.time_since_grounded < config.coyote_time;

                if is_within_buffer && can_jump_now {
                    Some((
                        e,
                        *config,
                        orientation.copied().unwrap_or_default(),
                        controller.clone(),
                    ))
                } else {
                    None
                }
            })
            .collect();

    for (entity, config, orientation, controller) in entities {
        // Consume the jump request by taking it from MovementIntent
        if let Some(mut intent) = world.get_mut::<MovementIntent>(entity) {
            intent.take_jump_request();
        }

        // Calculate jump direction
        let up = if controller.ground_detected() {
            controller.ground_normal()
        } else {
            orientation.up()
        };

        // Apply jump impulse
        // jump_speed is the desired velocity change. Impulse = mass * delta_v
        // When config.mass is Some, we scale by actual mass to get consistent jump height.
        // When None, we just apply jump_speed as the impulse (Rapier will divide by mass).
        let impulse = match config.mass {
            Some(_) => {
                // Scale impulse so velocity change equals jump_speed
                let actual_mass = B::get_mass(world, entity);
                up * config.jump_speed * actual_mass
            }
            None => {
                // No scaling - apply as impulse directly
                // Note: Rapier divides by mass, so velocity change = jump_speed / mass
                up * config.jump_speed
            }
        };
        B::apply_impulse(world, entity, impulse);
    }
}

/// Apply upright torque to keep characters oriented correctly.
///
/// Uses a cubic spring-damper system to orient the character to the target rotation.
/// The torque formula is: `(angle_error³ * spring_strength) - (angular_velocity * damping)`
/// The cubic term provides stronger correction at large angles.
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
        let current_rotation = B::get_rotation(world, entity);
        let angular_velocity = B::get_angular_velocity(world, entity);

        let target_angle = config
            .upright_target_angle
            .unwrap_or_else(|| orientation.angle() - consts::FRAC_PI_2);

        // Calculate angle error (shortest rotation to goal), normalized to [-PI, PI]
        let mut angle_error = target_angle - current_rotation;
        while angle_error > consts::PI {
            angle_error -= consts::TAU;
        }
        while angle_error < -consts::PI {
            angle_error += consts::TAU;
        }

        // Get inertia for scaling torque
        // Scale by actual inertia so torque produces consistent angular acceleration
        let actual_inertia = B::get_principal_inertia(world, entity);

        // Apply cubic spring-damper torque for stronger correction at large angles
        // Scale by inertia so config values work consistently across different body shapes
        let spring_torque = config.upright_torque_strength
            * angle_error
            * angle_error
            * angle_error.signum()
            * actual_inertia;
        let damping_torque = -config.upright_torque_damping * angular_velocity * actual_inertia;

        let total_torque = spring_torque + damping_torque;

        // Clamp total torque to prevent overflow.
        // Maximum torque is based on the maximum spring torque at full rotation error (PI).
        // This prevents the damping term from creating runaway torque when
        // spinning at high angular velocity.
        let max_spring_torque =
            config.upright_torque_strength * consts::PI * consts::PI * actual_inertia;
        let max_torque = max_spring_torque * 3.0;
        let clamped_torque = total_torque.clamp(-max_torque, max_torque);

        B::apply_torque(world, entity, clamped_torque);
    }
}
