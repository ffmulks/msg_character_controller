//! Core controller systems.
//!
//! These systems implement the floating character controller behavior.
//! They are generic over the physics backend to allow different physics
//! engines to be used.
//!
//! # System Order
//!
//! The systems are designed to run in a specific order:
//!
//! 1. **Preparation**: Clear forces from previous frame
//! 2. **Intent Evaluation**: Read MovementIntent and set intent flags
//! 3. **Sensors**: Collect ground, wall, ceiling data (can run in parallel)
//! 4. **Force Accumulation**: Spring, gravity, stair climb, upright torque
//! 5. **Intent Application**: Apply jump, walk, fly based on intent
//! 6. **Final Application**: Apply accumulated forces to physics
//!
//! Within each phase, systems that don't depend on each other can run in parallel.

use std::f32::consts;

use bevy::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, CharacterOrientation, ControllerConfig};
use crate::intent::MovementIntent;

// ============================================================================
// PHASE 2: INTENT EVALUATION
// ============================================================================

/// Evaluate MovementIntent and set intent flags on CharacterController.
///
/// This system runs early in the frame to determine:
/// - Whether the character intends to propel upward (jump or fly up)
///
/// These flags are used by downstream systems (like the spring) to make
/// decisions before the actual forces are applied.
pub fn evaluate_intent<B: CharacterPhysicsBackend>(
    time: Res<Time>,
    mut query: Query<(&mut CharacterController, &ControllerConfig, &MovementIntent)>,
) {
    let current_time = time.elapsed_secs();

    for (mut controller, config, intent) in &mut query {
        // Reset intent state for this frame
        controller.reset_intent_state();

        // Check if intending to jump (has valid jump request and can jump)
        let intends_jump = if let Some(ref jump) = intent.jump_request {
            let is_within_buffer = jump.is_within_buffer(current_time, config.jump_buffer_time);
            let can_jump = controller.is_grounded(config)
                || controller.time_since_grounded < config.coyote_time;
            is_within_buffer && can_jump
        } else {
            false
        };

        // Check if intending to fly upward
        let intends_fly_up = intent.fly > 0.001;

        // Set the combined upward propulsion intent
        controller.intends_upward_propulsion = intends_jump || intends_fly_up;
    }
}

// ============================================================================
// PHASE 4: FORCE ACCUMULATION
// ============================================================================

/// Accumulate floating spring force to maintain riding height.
///
/// Simple spring-damper: F = k * displacement - c * velocity
/// - displacement = target_height - current_height (positive = below target)
/// - velocity = vertical velocity (positive = moving up)
///
/// When climbing stairs, the target height includes `active_stair_height` to
/// temporarily raise the character over the step.
///
/// Downward spring forces are filtered when:
/// - `intends_upward_propulsion` is true (same-frame filtering)
/// - Within `jump_spring_filter_duration` after propulsion (cross-frame filtering)
///
/// Upward spring forces always remain active.
pub fn accumulate_spring_force<B: CharacterPhysicsBackend>(world: &mut World) {
    let time = world
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0);

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

        // Target height includes active_stair_height when climbing stairs
        // This temporarily raises the riding height to lift the character over the step
        let target_height = controller.effective_riding_height(&config);
        let current_height = floor.distance;

        // Check if we should filter downward spring forces:
        // 1. intends_upward_propulsion - intent evaluated this frame (same-frame filtering)
        // 2. in_jump_spring_filter_window - time-based filter (cross-frame filtering)
        let should_filter_downward = controller.intends_upward_propulsion
            || controller.in_jump_spring_filter_window(time, config.jump_spring_filter_duration);

        // Only apply spring within active range, unless filtering
        // During filtering, we still process the spring but filter downward forces
        let max_range = target_height + config.ground_tolerance;
        let min_range = controller.capsule_half_height();
        if current_height < min_range {
            // Below minimum range (physics collision zone) - skip spring
            continue;
        }
        if current_height > max_range && !should_filter_downward {
            // Above max range and not filtering - skip spring entirely
            continue;
        }

        // Spring-damper formula: F = k * x - c * v
        // x = displacement from target (positive = below target, needs push up)
        // v = vertical velocity (positive = moving up, damp it)
        let displacement = target_height - current_height;

        // Check if already moving toward target fast enough (velocity clamp).
        // If moving in the correct direction at or above max velocity, skip force application.
        if let Some(max_vel) = config.spring_max_velocity {
            // Positive displacement means we need positive velocity (upward) to correct
            let moving_toward_target = (displacement > 0.0 && vertical_velocity > 0.0)
                || (displacement < 0.0 && vertical_velocity < 0.0);
            if moving_toward_target && vertical_velocity.abs() >= max_vel {
                continue;
            }
        }

        // Get mass for force scaling
        // Scale spring force by mass so config values produce consistent acceleration
        let mass = B::get_mass(world, entity);

        let spring_force = (config.spring_strength * displacement
            - config.spring_damping * vertical_velocity)
            * mass;

        // Clamp spring force to configured max, or use formula-based fallback.
        // The fallback prevents overflow when entering the spring zone at high velocity.
        let gravity_magnitude = controller.gravity.length();
        let max_spring_force = config
            .spring_max_force
            .map(|f| f * mass)
            .unwrap_or_else(|| {
                // Fallback: maximum based on counteracting gravity force plus reasonable acceleration.
                // F = m * g, so max force = m * g * 3 + spring contribution
                gravity_magnitude * mass * 3.0
                    + config.spring_strength * config.ground_tolerance * mass
            });
        let clamped_spring_force = spring_force.clamp(-max_spring_force, max_spring_force);

        // When upward propulsion is intended or within filter window, reject downward spring forces
        // (negative force would push character down, counteracting the jump/fly)
        // Upward spring forces remain active to help correct position
        let final_spring_force = if should_filter_downward && clamped_spring_force < 0.0 {
            // Filter out downward force during upward propulsion
            0.0
        } else {
            clamped_spring_force
        };

        // Apply force along up direction
        let force = up * final_spring_force;
        B::apply_force(world, entity, force);
    }
}

/// Accumulate stair climbing forces and update active_stair_height.
///
/// When a step is detected that is higher than the float height but within
/// the max climb height, this system:
/// 1. Sets `active_stair_height` to the measured step height (used by the spring system)
/// 2. Applies extra upward force based on max_spring_force * climb_force_multiplier
///
/// Using max_spring_force provides responsive climbing since it represents the
/// maximum force the spring system can apply.
///
/// When no step is detected, `active_stair_height` is reset to 0.
pub fn accumulate_stair_climb_force<B: CharacterPhysicsBackend>(world: &mut World) {
    // Collect entities with stair config
    let entities: Vec<(Entity, ControllerConfig, CharacterController, CharacterOrientation)> = world
        .query::<(
            Entity,
            &ControllerConfig,
            &CharacterController,
            Option<&CharacterOrientation>,
        )>()
        .iter(world)
        .filter(|(_, _, controller, _)| controller.stair_stepping_enabled())
        .map(|(e, config, controller, orientation)| {
            (
                e,
                *config,
                controller.clone(),
                orientation.copied().unwrap_or_default(),
            )
        })
        .collect();

    for (entity, config, controller, orientation) in entities {
        // Get stair config (we know it exists because of the filter)
        let stair_config = controller.stair_config.as_ref().unwrap();

        // Check if a step is detected that requires climbing
        let should_climb = controller.step_detected
            && controller.step_height > config.float_height + stair_config.stair_tolerance
            && controller.step_height <= stair_config.max_climb_height;

        if should_climb {
            // Set active_stair_height to the measured step height
            if let Some(mut ctrl) = world.get_mut::<CharacterController>(entity) {
                ctrl.active_stair_height = controller.step_height;
            }

            // Apply extra upward force to help climb the step
            // Use max_spring_force * multiplier for responsive climbing
            let mass = B::get_mass(world, entity);
            let gravity_magnitude = controller.gravity.length();
            let up = orientation.up();

            // Calculate max spring force (same formula as floating spring system)
            let max_spring_force = config
                .spring_max_force
                .map(|f| f * mass)
                .unwrap_or_else(|| {
                    gravity_magnitude * mass * 3.0
                        + config.spring_strength * config.ground_tolerance * mass
                });

            let climb_force = up * max_spring_force * stair_config.climb_force_multiplier;

            B::apply_force(world, entity, climb_force);
        } else {
            // No step detected or step is not climbable - reset active_stair_height
            if let Some(mut ctrl) = world.get_mut::<CharacterController>(entity) {
                // Gradually decay active_stair_height for smooth transition
                // Or reset immediately - for now, reset immediately
                ctrl.active_stair_height = 0.0;
            }
        }
    }
}

/// Accumulate gravity impulse.
///
/// Gravity is applied from CharacterController.gravity as an impulse when the
/// character is not grounded. Applied as an impulse each physics frame to
/// produce the equivalent acceleration.
///
/// Note: Gravity is always applied internally by this system. To change the
/// gravity affecting a character, modify CharacterController::gravity directly.
pub fn accumulate_gravity<B: CharacterPhysicsBackend>(world: &mut World) {
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
        let mass = B::get_mass(world, entity);
        let gravity_impulse = controller.gravity * mass * dt;
        B::apply_impulse(world, entity, gravity_impulse);
    }
}

// ============================================================================
// PHASE 5: INTENT APPLICATION
// ============================================================================

/// Apply walking movement based on intent.
///
/// Handles horizontal movement using impulses scaled by mass for consistent
/// acceleration. Uses slope tangent when grounded, world-space horizontal when airborne.
///
/// For the floating controller, friction is simulated internally since the character
/// hovers above the ground and doesn't use Rapier's contact friction.
pub fn apply_walk<B: CharacterPhysicsBackend>(world: &mut World) {
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

    let dt = B::get_fixed_timestep(world);

    for (entity, config, orientation, intent, controller) in entities {
        let current_velocity = B::get_velocity(world, entity);
        let mass = B::get_mass(world, entity);
        let right = orientation.right();

        let is_grounded = controller.is_grounded(&config);

        let desired_walk_speed = intent.effective_walk() * config.max_speed;
        let walk_accel = if is_grounded {
            config.acceleration
        } else {
            config.acceleration * config.air_control
        };

        if is_grounded && controller.ground_detected() {
            // GROUNDED: Move along slope surface using forces
            let slope_tangent = controller.ground_tangent();
            let current_slope_speed = current_velocity.dot(slope_tangent);

            // Calculate velocity change toward target, clamped by max acceleration
            let velocity_diff = desired_walk_speed - current_slope_speed;
            let max_change = walk_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            // Apply friction when not walking
            let friction_factor = if !intent.is_walking() {
                1.0 - config.friction
            } else {
                1.0
            };
            let new_slope_speed = (current_slope_speed + change) * friction_factor;
            let slope_velocity_delta = new_slope_speed - current_slope_speed;

            // Apply impulse along slope tangent: I = m * dv
            let walk_impulse = slope_tangent * slope_velocity_delta * mass;
            B::apply_impulse(world, entity, walk_impulse);

            // Dampen normal velocity: preserve 50% of downward motion, zero out upward
            let slope_normal = controller.ground_normal();
            let normal_velocity = current_velocity.dot(slope_normal);
            let target_normal = if normal_velocity < 0.0 {
                normal_velocity * 0.5
            } else {
                0.0
            };
            let normal_velocity_delta = target_normal - normal_velocity;
            let normal_impulse = slope_normal * normal_velocity_delta * mass;
            B::apply_impulse(world, entity, normal_impulse);
        } else {
            // AIRBORNE: Use world-space horizontal axis
            let current_horizontal = current_velocity.dot(right);

            // Calculate velocity change toward target, clamped by max acceleration
            let velocity_diff = desired_walk_speed - current_horizontal;
            let max_change = walk_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            // Apply impulse along right axis: I = m * dv
            let walk_impulse = right * change * mass;
            B::apply_impulse(world, entity, walk_impulse);
        }
    }
}

/// Apply vertical propulsion (flying) based on intent.
///
/// Handles vertical movement using impulses scaled by mass. Upward propulsion
/// is boosted by gravity magnitude to counteract gravity.
///
/// Flying downwards is disabled while grounded.
pub fn apply_fly<B: CharacterPhysicsBackend>(world: &mut World) {
    let time = world
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0);

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

    let dt = B::get_fixed_timestep(world);

    for (entity, config, orientation, intent, controller) in entities {
        let is_grounded = controller.is_grounded(&config);

        // Flying downwards is disabled while grounded
        let fly_direction = intent.fly;
        let should_apply_fly = intent.is_flying() && !(is_grounded && intent.is_flying_down());

        if !should_apply_fly {
            continue;
        }

        let current_velocity = B::get_velocity(world, entity);
        let mass = B::get_mass(world, entity);
        let up = orientation.up();

        let current_vertical = current_velocity.dot(up);
        let desired_vertical = intent.effective_fly() * config.max_speed;

        let mut fly_accel = config.acceleration;
        // Boost upward propulsion by gravity magnitude to counteract gravity
        if fly_direction > 0.0 {
            fly_accel += controller.gravity.length();
        }

        // Calculate velocity change toward target, clamped by max acceleration
        let velocity_diff = desired_vertical - current_vertical;
        let max_change = fly_accel * dt;
        let change = velocity_diff.clamp(-max_change, max_change);

        // Apply impulse along up axis: I = m * dv
        let fly_impulse = up * change * mass;
        B::apply_impulse(world, entity, fly_impulse);

        // Record upward propulsion time when actively flying up
        if fly_direction > 0.0 && change > 0.0 {
            if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
                controller.record_upward_propulsion(time);
            }
        }
    }
}

/// Apply movement (walking and flying) based on intent.
///
/// **DEPRECATED**: Use `apply_walk` and `apply_fly` separately for better control.
///
/// This unified system handles both horizontal walking and vertical propulsion
/// using impulses rather than direct velocity manipulation. All impulses are scaled
/// by mass for consistent acceleration regardless of character mass.
///
/// For the floating controller, friction is simulated internally since the character
/// hovers above the ground and doesn't use Rapier's contact friction.
///
/// Flying downwards is disabled while grounded.
#[deprecated(since = "0.3.0", note = "Use apply_walk and apply_fly separately")]
pub fn apply_movement<B: CharacterPhysicsBackend>(world: &mut World) {
    let time = world
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0);

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
    let dt = B::get_fixed_timestep(world);

    for (entity, config, orientation, intent, controller) in entities {
        let current_velocity = B::get_velocity(world, entity);
        let mass = B::get_mass(world, entity);
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

        if is_grounded && controller.ground_detected() {
            // GROUNDED: Move along slope surface using forces
            let slope_tangent = controller.ground_tangent();
            let current_slope_speed = current_velocity.dot(slope_tangent);

            // Calculate velocity change toward target, clamped by max acceleration
            let velocity_diff = desired_walk_speed - current_slope_speed;
            let max_change = walk_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            // Apply friction when not walking
            let friction_factor = if !intent.is_walking() {
                1.0 - config.friction
            } else {
                1.0
            };
            let new_slope_speed = (current_slope_speed + change) * friction_factor;
            let slope_velocity_delta = new_slope_speed - current_slope_speed;

            // Apply impulse along slope tangent: I = m * dv
            let walk_impulse = slope_tangent * slope_velocity_delta * mass;
            B::apply_impulse(world, entity, walk_impulse);

            // Dampen normal velocity: preserve 50% of downward motion, zero out upward
            let slope_normal = controller.ground_normal();
            let normal_velocity = current_velocity.dot(slope_normal);
            let target_normal = if normal_velocity < 0.0 {
                normal_velocity * 0.5
            } else {
                0.0
            };
            let normal_velocity_delta = target_normal - normal_velocity;
            let normal_impulse = slope_normal * normal_velocity_delta * mass;
            B::apply_impulse(world, entity, normal_impulse);
        } else {
            // AIRBORNE: Use world-space horizontal axis
            let current_horizontal = current_velocity.dot(right);

            // Calculate velocity change toward target, clamped by max acceleration
            let velocity_diff = desired_walk_speed - current_horizontal;
            let max_change = walk_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            // Apply impulse along right axis: I = m * dv
            let walk_impulse = right * change * mass;
            B::apply_impulse(world, entity, walk_impulse);
        }

        // === FLYING ===
        // Flying downwards is disabled while grounded
        let fly_direction = intent.fly;
        let should_apply_fly = intent.is_flying() && !(is_grounded && intent.is_flying_down());

        if should_apply_fly {
            let current_vertical = current_velocity.dot(up);
            let desired_vertical = intent.effective_fly() * config.max_speed;

            let mut fly_accel = config.acceleration;
            // Boost upward propulsion by gravity magnitude to counteract gravity
            if fly_direction > 0.0 {
                fly_accel += controller.gravity.length();
            }

            // Calculate velocity change toward target, clamped by max acceleration
            let velocity_diff = desired_vertical - current_vertical;
            let max_change = fly_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            // Apply impulse along up axis: I = m * dv
            let fly_impulse = up * change * mass;
            B::apply_impulse(world, entity, fly_impulse);

            // Record upward propulsion time when actively flying up
            if fly_direction > 0.0 && change > 0.0 {
                if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
                    controller.record_upward_propulsion(time);
                }
            }
        }
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
    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterOrientation,
        CharacterController,
    )> = world
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
        // Scale by actual mass so velocity change equals jump_speed regardless of body mass.
        let mass = B::get_mass(world, entity);
        let impulse = up * config.jump_speed * mass;
        B::apply_impulse(world, entity, impulse);

        // Record upward propulsion time for spring force filtering
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.record_upward_propulsion(time);
        }
    }
}

/// Accumulate upright torque to keep characters oriented correctly.
///
/// Uses a simple linear spring-damper system to orient the character to the target rotation.
/// The torque formula is: `(strength * angle_error) - (damping * angular_velocity)`
/// Both terms are scaled by inertia for consistent behavior across different body shapes.
///
/// For critical damping (no oscillation): `damping = 2 * sqrt(strength)`
pub fn accumulate_upright_torque<B: CharacterPhysicsBackend>(world: &mut World) {
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
        let inertia = B::get_principal_inertia(world, entity);

        // Apply simple linear spring-damper torque.
        // This is a standard second-order system: torque = -k*θ - c*ω
        // where k = spring strength, c = damping coefficient, θ = angle error, ω = angular velocity.
        // Scale by inertia so config values work consistently across different body shapes.
        let damping_torque = -config.upright_torque_damping * angular_velocity * inertia;

        // Check if already rotating toward target fast enough (velocity clamp).
        // If at or above max velocity, only apply damping (no spring) to slow down.
        // This prevents the spring from adding more acceleration when we're already fast enough.
        let spring_torque = if let Some(max_vel) = config.upright_max_angular_velocity {
            let rotating_toward_target = (angle_error > 0.0 && angular_velocity > 0.0)
                || (angle_error < 0.0 && angular_velocity < 0.0);
            if rotating_toward_target && angular_velocity.abs() >= max_vel {
                // At max velocity - don't add more spring force, just let damping work
                0.0
            } else {
                config.upright_torque_strength * angle_error * inertia
            }
        } else {
            config.upright_torque_strength * angle_error * inertia
        };

        let total_torque = spring_torque + damping_torque;

        // Clamp total torque to configured max, or use formula-based fallback.
        let max_torque = config
            .upright_max_torque
            .map(|t| t * inertia)
            .unwrap_or_else(|| {
                // Fallback: maximum based on spring torque at full rotation error (PI).
                let max_spring_torque = config.upright_torque_strength * consts::PI * inertia;
                max_spring_torque * 3.0
            });
        let clamped_torque = total_torque.clamp(-max_torque, max_torque);

        B::apply_torque(world, entity, clamped_torque);
    }
}
