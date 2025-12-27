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
/// This system reads ground detection data from the CharacterController
/// and applies spring forces to maintain the configured float height.
/// On slopes, it also applies forces to keep the character snapped to the
/// ground surface and prevent bouncing.
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
        if !controller.ground_detected {
            continue;
        }

        let velocity = B::get_velocity(world, entity);
        let up = orientation.up();
        let gravity = controller.gravity;

        // Use effective float height which accounts for collider dimensions
        let effective_height = controller.effective_float_height(&config);

        // Calculate height error (positive = below float height, negative = above)
        let height_error = controller.height_error(effective_height);

        // Get velocity component perpendicular to ground (along ground normal)
        // This is more accurate for slopes than using orientation.up()
        let ground_normal = controller.ground_normal;
        let normal_velocity = velocity.dot(ground_normal);

        // Float height is a FLOOR, not a target!
        // Only apply spring force when BELOW float height (height_error > 0)
        // Never pull the character DOWN when they're above float height (e.g., jumping)
        if height_error > 0.0 {
            // When moving downward and below float height, we need to actively push up.
            // The spring force alone may not be strong enough to counteract deliberate
            // downward propulsion, so we also apply a velocity correction.
            if normal_velocity < 0.0 {
                // Moving toward ground while below float height - apply velocity correction
                // This ensures the character is pushed UP to float height, not just prevented
                // from falling. The correction is proportional to how far below we are.
                let correction_strength = (height_error / effective_height).min(1.0);
                let velocity_correction = -normal_velocity * correction_strength;

                // Apply both the velocity correction (immediate) and spring force (smooth)
                let corrected_velocity = velocity + ground_normal * velocity_correction;
                B::set_velocity(world, entity, corrected_velocity);
            }

            // Spring force: F = k * x - c * v
            // Damping uses velocity perpendicular to ground for better slope handling
            let spring_force =
                config.spring_strength * height_error - config.spring_damping * normal_velocity;

            let force = up * spring_force;
            B::apply_force(world, entity, force);
        }

        // SLOPE SNAPPING: Apply additional downward force on slopes to keep
        // the character pressed against the ground surface. This prevents
        // the character from bouncing or floating off when walking on slopes.
        if controller.is_grounded && controller.slope_angle > 0.05 {
            // The steeper the slope, the more we need to snap
            let slope_factor = controller.slope_angle.sin();

            // Only apply snap force when:
            // 1. Not jumping (normal velocity not too high)
            // 2. Within reasonable distance of target height
            if normal_velocity < 50.0 && height_error.abs() < config.cling_distance * 2.0 {
                // Apply a force toward the ground along the ground normal
                // This helps keep the character pressed against the slope
                let snap_strength = config.spring_strength * 0.3 * slope_factor;
                let snap_force = -ground_normal * snap_strength;
                B::apply_force(world, entity, snap_force);
            }
        }

        // Apply cling force when above float height but within cling distance
        // This helps the character stick to ground on bumpy terrain
        // But skip if we're moving upward fast (probably jumping)
        if height_error < 0.0
            && controller.ground_distance <= effective_height + config.cling_distance
            && config.cling_strength > 0.0
            && normal_velocity < 100.0
        {
            let cling_force = gravity * config.cling_strength;
            B::apply_force(world, entity, cling_force);
        }

        // Apply extra fall gravity when falling (velocity is in -UP direction)
        // This makes the character fall faster, creating a more responsive feel
        // Uses the controller's gravity field
        let vertical_velocity = velocity.dot(up);
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
        .filter(|(_, controller)| !controller.is_grounded)
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
///
/// When grounded on slopes, velocity is projected onto the slope surface to prevent
/// the character from launching into the air or sliding off the slope. The floating
/// spring system handles maintaining the correct height above the ground.
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
        let right = orientation.right();

        // Calculate desired speed from input
        let desired_speed = intent.effective() * config.max_speed;

        // Determine acceleration based on grounded state
        let accel = if controller.is_grounded {
            config.acceleration
        } else {
            config.acceleration * config.air_control
        };

        let new_velocity = if controller.is_grounded && controller.ground_detected {
            // GROUNDED: Move along slope surface
            //
            // The key insight: when grounded, movement should be ENTIRELY along the
            // slope surface (tangent). The floating spring handles the vertical component.
            // We should NOT preserve world-space vertical velocity - that causes launching.

            let slope_tangent = controller.ground_tangent();
            let slope_normal = controller.ground_normal;

            // Project current velocity onto the slope surface to get our current speed
            // along the slope. This removes any velocity perpendicular to the slope.
            let current_slope_speed = current_velocity.dot(slope_tangent);

            // Calculate velocity change to reach desired speed
            let velocity_diff = desired_speed - current_slope_speed;
            let max_change = accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            // Apply friction when no input
            let friction_multiplier = if !intent.is_active() {
                1.0 - config.friction
            } else {
                1.0
            };

            let new_slope_speed = (current_slope_speed + change) * friction_multiplier;

            // The new velocity is entirely along the slope tangent.
            // The spring system will handle pushing us up/down to maintain float height.
            // We keep a small amount of the normal component for the spring to work with,
            // but heavily damped to prevent bouncing.
            let normal_velocity = current_velocity.dot(slope_normal);

            // Only preserve downward normal velocity (toward the slope) slightly,
            // zero out upward normal velocity to prevent launching
            let preserved_normal = if normal_velocity < 0.0 {
                // Moving toward slope - preserve a bit for ground conformance
                normal_velocity * 0.5
            } else {
                // Moving away from slope - zero it out to prevent launching
                0.0
            };

            slope_tangent * new_slope_speed + slope_normal * preserved_normal
        } else {
            // AIRBORNE: Use world-space coordinates
            //
            // In the air, we use the character's local coordinate system.
            // Horizontal movement is along the right direction, vertical is preserved.

            // Get horizontal component of current velocity (along character's right)
            let current_horizontal = current_velocity.dot(right);

            // Calculate velocity change
            let velocity_diff = desired_speed - current_horizontal;
            let max_change = accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            // No friction in air
            let new_horizontal = current_horizontal + change;

            // Preserve vertical velocity (gravity handles this)
            let vertical_velocity = current_velocity.dot(up);

            right * new_horizontal + up * vertical_velocity
        };

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
    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation, PropulsionIntent, CharacterController)> = world
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
