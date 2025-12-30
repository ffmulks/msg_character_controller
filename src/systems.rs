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
//! 2. **Sensors**: Collect ground, wall, ceiling data (can run in parallel)
//! 3. **Intent Evaluation**: Read MovementIntent and set intent flags (requires sensor data)
//! 4. **Force Accumulation**: Spring, gravity, stair climb, upright torque
//! 5. **Intent Application**: Apply jump, walk, fly based on intent
//! 6. **Final Application**: Apply accumulated forces to physics
//!
//! Within each phase, systems that don't depend on each other can run in parallel.

use std::f32::consts;
use std::time::Duration;

use bevy::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, ControllerConfig};
use crate::intent::MovementIntent;

// ============================================================================
// PHASE 1: PREPARATION (Timer Ticking)
// ============================================================================

/// Tick jump request timers for all entities.
///
/// This system runs early in the frame to advance all jump request timers.
/// It must run before `expire_jump_requests` which removes expired requests.
pub fn tick_jump_request_timers(time: Res<Time>, mut query: Query<&mut MovementIntent>) {
    let delta = time.delta();
    for mut intent in &mut query {
        if let Some(ref mut jump) = intent.jump_request {
            jump.tick(delta);
        }
    }
}

/// Remove expired jump requests.
///
/// This system runs after `tick_jump_request_timers` and before `evaluate_intent`.
/// It removes jump requests whose buffer timer has finished, so that downstream
/// systems don't need to check buffer validity.
pub fn expire_jump_requests(mut query: Query<&mut MovementIntent>) {
    for mut intent in &mut query {
        if let Some(ref jump) = intent.jump_request {
            if !jump.is_valid() {
                intent.jump_request = None;
            }
        }
    }
}

// ============================================================================
// PHASE 3: INTENT EVALUATION
// ============================================================================

/// Update coyote timer and jump spring filter timer.
///
/// This system runs in IntentEvaluation phase before intent is evaluated,
/// ensuring timers are current when checking jump validity.
///
/// - Coyote timer: Reset when grounded OR touching wall (if wall jumping enabled)
/// - Jump spring filter timer: Always tick
pub fn update_timers(
    time: Res<Time<Fixed>>,
    mut query: Query<(&mut CharacterController, &ControllerConfig)>,
) {
    let delta = Duration::from_secs_f64(time.delta_secs_f64());

    for (mut controller, config) in &mut query {
        // Check if we have valid contact for coyote time
        let is_grounded = controller.is_grounded(config);
        let has_wall_contact = config.wall_jumping && controller.touching_wall();

        // Reset coyote timer when we have any valid contact
        if is_grounded || has_wall_contact {
            controller.reset_coyote_timer(config.coyote_time);
        } else {
            controller.tick_coyote_timer(delta);
        }

        // Tick the jump spring filter timer
        controller.jump_spring_filter_timer.tick(delta);

        // Tick the jumped timer (for fall gravity tracking)
        controller.jumped_timer.tick(delta);

        // Tick the extra gravity timer
        controller.extra_gravity_timer.tick(delta);
    }
}

/// Update the jump type based on current contact state.
///
/// This system determines which type of jump should be performed based on
/// ground and wall contact. The detection priority is:
/// 1. Ground contact -> JumpType::Ground (normal jump)
/// 2. Only left wall contact -> JumpType::LeftWall (wall jump)
/// 3. Only right wall contact -> JumpType::RightWall (wall jump)
///
/// This runs after sensors and timers, before evaluate_intent, so that
/// buffered jumps will process the correct jump type when contact is made.
/// The last_jump_type is stored on the controller for coyote time to respect.
pub fn update_jump_type(
    mut query: Query<(&mut CharacterController, &ControllerConfig)>,
) {
    use crate::config::JumpType;

    for (mut controller, config) in &mut query {
        // Ground contact takes priority - normal jump
        if controller.is_grounded(config) {
            controller.last_jump_type = JumpType::Ground;
            continue;
        }

        // Check wall contact (only if wall jumping is enabled)
        if config.wall_jumping {
            let touching_left = controller.touching_left_wall();
            let touching_right = controller.touching_right_wall();

            // Only one wall contact = wall jump
            // Both walls or no walls = keep previous type (for coyote time)
            if touching_left && !touching_right {
                controller.last_jump_type = JumpType::LeftWall;
            } else if touching_right && !touching_left {
                controller.last_jump_type = JumpType::RightWall;
            }
            // If touching both walls or neither, keep the previous jump type
            // This allows coyote time to work correctly
        }
    }
}

/// Evaluate MovementIntent and set intent flags on CharacterController.
///
/// This system runs AFTER sensors to have access to current frame's floor data.
/// It determines:
/// - Whether the character intends to propel upward (jump or fly up)
///
/// These flags are used by downstream systems (like the spring) to make
/// decisions before the actual forces are applied. Running after sensors
/// ensures that `is_grounded` checks use current frame data, so that
/// `intends_upward_propulsion` is correctly set when landing and jumping
/// in the same frame.
///
/// Note: Expired jump requests are removed by `expire_jump_requests` before
/// this system runs, so we just check if a request exists.
pub fn evaluate_intent<B: CharacterPhysicsBackend>(
    mut query: Query<(&mut CharacterController, &ControllerConfig, &MovementIntent)>,
) {
    use crate::config::JumpType;

    for (mut controller, config, intent) in &mut query {
        // Reset intent state for this frame
        controller.reset_intent_state();

        // Check if intending to jump (has valid jump request and can jump)
        // Expired requests are already removed by expire_jump_requests
        // Can jump if: grounded, touching wall (with wall jumping), or within coyote time
        let has_contact = controller.is_grounded(config)
            || (config.wall_jumping && controller.touching_wall());
        let can_jump = has_contact || controller.in_coyote_time();

        // For wall jumps via coyote time, also check that last_jump_type is a wall type
        let valid_jump_type = match controller.last_jump_type {
            JumpType::Ground => true,
            JumpType::LeftWall | JumpType::RightWall => config.wall_jumping,
        };

        let intends_jump = intent.jump_request.is_some() && can_jump && valid_jump_type;

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
/// **Grounding strength**: When the character is slightly above the target height
/// (within `grounding_distance`), the downward spring force is multiplied by
/// `grounding_strength`. This helps keep the character firmly grounded when
/// floating slightly above the riding height.
///
/// Downward spring forces are filtered when:
/// - `intends_upward_propulsion` is true (same-frame filtering)
/// - Within `jump_spring_filter_duration` after propulsion (cross-frame filtering)
///
/// Upward spring forces always remain active.
pub fn accumulate_spring_force<B: CharacterPhysicsBackend>(world: &mut World) {

    let entities: Vec<(Entity, ControllerConfig, CharacterController)> = world
        .query::<(Entity, &ControllerConfig, &CharacterController)>()
        .iter(world)
        .map(|(e, config, controller)| (e, *config, controller.clone()))
        .collect();

    for (entity, config, controller) in entities {
        let Some(ref floor) = controller.floor else {
            continue;
        };

        if controller.upward_intent() {
            continue;
        }

        let up = controller.ideal_up();
        let velocity = B::get_velocity(world, entity);
        let vertical_velocity = velocity.dot(up);

        // Target height includes active_stair_height when climbing stairs
        // This temporarily raises the riding height to lift the character over the step
        let target_height = controller.effective_riding_height(&config);
        let current_height = floor.distance;

        // Check if we should filter downward spring forces
        let should_filter_downward = controller.upward_intent();

        // Only apply spring within active range, unless filtering
        // During filtering, we still process the spring but filter downward forces
        let max_range = target_height + config.ground_tolerance;

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

        let mut spring_force = (config.spring_strength * displacement
            - config.spring_damping * vertical_velocity)
            * mass;

        // Apply grounding_strength multiplier when character is slightly above target
        // (within grounding_distance buffer zone) and spring is pushing down.
        // This helps keep the character grounded by amplifying the downward pull.
        // displacement < 0 means current_height > target_height (above target)
        // -displacement gives the height above target
        // Only apply when velocity is pointing up - if already falling, natural spring handles it
        let height_above_target = -displacement;
        if displacement < 0.0
            && height_above_target <= config.grounding_distance
            && spring_force < 0.0
            && vertical_velocity > 0.0
        {
            spring_force *= config.grounding_strength;
        }

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
    let entities: Vec<(Entity, ControllerConfig, CharacterController)> = world
        .query::<(Entity, &ControllerConfig, &CharacterController)>()
        .iter(world)
        .filter(|(_, _, controller)| controller.stair_stepping_enabled())
        .map(|(e, config, controller)| (e, *config, controller.clone()))
        .collect();

    for (entity, config, controller) in entities {
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
            let up = controller.ideal_up();

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
/// **Important**: Gravity is filtered during upward propulsion (jump or fly up)
/// to prevent fighting against the intended upward movement. This filtering
/// uses the same `jump_spring_filter_window` as the spring system for consistency.
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

/// Apply extra fall gravity for early jump cancellation.
///
/// This system enables players to cancel jumps early by releasing the jump button,
/// making jumps feel less floaty. Extra gravity is triggered when:
///
/// 1. We jumped recently (within `jump_cancel_window`)
/// 2. AND either:
///    - No jump request is pending this frame (player let go of button)
///    - OR we're moving downward (crossed the zenith)
///
/// When triggered, extra fall gravity is applied for `extra_gravity_duration`,
/// multiplying gravity by `extra_fall_gravity`.
///
/// This system MUST run before `apply_jump` to check for jump requests before
/// they are consumed.
pub fn apply_fall_gravity<B: CharacterPhysicsBackend>(world: &mut World) {
    let dt = B::get_fixed_timestep(world);

    // Collect entities that might need fall gravity
    let entities: Vec<(Entity, ControllerConfig, CharacterController, bool)> = world
        .query::<(
            Entity,
            &ControllerConfig,
            &CharacterController,
            &MovementIntent,
        )>()
        .iter(world)
        .filter(|(_, config, controller, _)| {
            // Only process airborne entities with extra_fall_gravity > 1.0
            !controller.is_grounded(config) && config.extra_fall_gravity > 1.0
        })
        .map(|(e, config, controller, intent)| {
            // Check if a jump request is pending (player is still holding jump)
            let has_jump_request = intent.jump_request.is_some();
            (e, *config, controller.clone(), has_jump_request)
        })
        .collect();

    for (entity, config, controller, has_jump_request) in entities {
        let up = controller.ideal_up();
        let velocity = B::get_velocity(world, entity);
        let vertical_velocity = velocity.dot(up);

        // Check if we should trigger extra fall gravity
        // Conditions:
        // 1. We jumped recently (within jump_cancel_window)
        // 2. AND either:
        //    - No jump request pending (player let go of button)
        //    - OR we're moving downward (crossed the zenith)
        let should_trigger = controller.in_jump_cancel_window()
            && (!has_jump_request || vertical_velocity < 0.0);

        // Trigger extra gravity if conditions are met
        if should_trigger && !controller.extra_fall_gravity_active() {
            if let Some(mut ctrl) = world.get_mut::<CharacterController>(entity) {
                ctrl.trigger_extra_fall_gravity(config.extra_gravity_duration);
            }
        }

        // Apply extra gravity if the timer is active
        // Re-check because we may have just triggered it
        let is_active = if let Some(ctrl) = world.get::<CharacterController>(entity) {
            ctrl.extra_fall_gravity_active()
        } else {
            false
        };

        if is_active {
            // Apply extra gravity impulse
            // The regular gravity system applies: gravity * mass * dt
            // We want to add: gravity * mass * dt * (extra_fall_gravity - 1)
            // This way total gravity becomes: gravity * mass * dt * extra_fall_gravity
            let mass = B::get_mass(world, entity);
            let extra_multiplier = config.extra_fall_gravity - 1.0;
            let extra_gravity_impulse = controller.gravity * mass * dt * extra_multiplier;
            B::apply_impulse(world, entity, extra_gravity_impulse);
        }
    }
}

/// Apply walking movement based on intent.
///
/// Handles horizontal movement using impulses scaled by mass for consistent
/// acceleration. Uses slope tangent when grounded, world-space horizontal when airborne.
///
/// For the floating controller, friction is simulated internally since the character
/// hovers above the ground and doesn't use Rapier's contact friction.
///
/// **Downhill walking**: When walking downhill (slope tangent has a downward component),
/// an extra downward impulse is applied based on `downhill_force_multiplier`. This helps
/// keep the character pressed against the slope instead of floating off due to momentum.
pub fn apply_walk<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(Entity, ControllerConfig, MovementIntent, CharacterController)> = world
        .query::<(
            Entity,
            &ControllerConfig,
            &MovementIntent,
            &CharacterController,
        )>()
        .iter(world)
        .map(|(e, config, intent, controller)| (e, *config, intent.clone(), controller.clone()))
        .collect();

    let dt = B::get_fixed_timestep(world);

    for (entity, config, intent, controller) in entities {
        let current_velocity = B::get_velocity(world, entity);
        let mass = B::get_mass(world, entity);
        let right = controller.ideal_right();

        let is_grounded = controller.is_grounded(&config);

        // Check for wall clinging rejection
        let effective_walk = intent.effective_walk();
        let walk_blocked = !config.wall_clinging
            && ((effective_walk > 0.0 && controller.touching_right_wall())
                || (effective_walk < 0.0 && controller.touching_left_wall()));

        let desired_walk_speed = if walk_blocked {
            0.0
        } else {
            effective_walk * config.max_speed
        };
        let walk_accel = if is_grounded {
            config.acceleration
        } else {
            config.acceleration * config.air_control
        };

        if is_grounded && controller.ground_detected() {
            // GROUNDED: Move along slope surface using forces
            // Clamp the slope tangent to respect max_slope_angle
            let up = controller.ideal_up();
            let slope_tangent = if controller.slope_angle <= config.max_slope_angle {
                // Within max slope angle, use actual slope tangent
                controller.ground_tangent()
            } else {
                // Slope exceeds max angle, clamp the tangent direction
                let ground_normal = controller.ground_normal();

                // Determine slope tilt direction based on which way the normal leans
                // If normal points away from right (negative dot), slope goes up when moving right
                let slope_tilt_sign = if ground_normal.dot(right) <= 0.0 {
                    1.0
                } else {
                    -1.0
                };

                // Rotate ideal_right by max_slope_angle in the tilt direction
                let cos_a = config.max_slope_angle.cos();
                let sin_a = config.max_slope_angle.sin() * slope_tilt_sign;

                // Compute clamped tangent: right rotated by max_slope_angle in the (right, up) plane
                Vec2::new(
                    right.x * cos_a + up.x * sin_a,
                    right.y * cos_a + up.y * sin_a,
                )
            };

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
            // Only the walking impulse is rotated - external velocity and spring system are unaffected
            let walk_impulse = slope_tangent * slope_velocity_delta * mass;
            B::apply_impulse(world, entity, walk_impulse);

            // Apply extra downhill force when walking downhill
            // This helps keep the character pressed against the slope
            if config.downhill_force_multiplier > 0.0 && intent.is_walking() {
                // Get the actual walk direction (slope tangent in the direction we're walking)
                let walk_sign = effective_walk.signum();
                let walk_direction = slope_tangent * walk_sign;

                // Check if walking downhill (walk direction has negative vertical component)
                let downhill_component = -walk_direction.dot(up);
                if downhill_component > 0.0 {
                    // Calculate downhill impulse proportional to:
                    // - The downhill angle (downhill_component)
                    // - The multiplier
                    // - Gravity magnitude
                    let down = -up;
                    let gravity_magnitude = controller.gravity.length();

                    // Extra downhill impulse = gravity * downhill_component * multiplier * dt
                    // This effectively adds extra gravity when walking downhill
                    let downhill_impulse =
                        down * gravity_magnitude * downhill_component * config.downhill_force_multiplier * mass * dt;
                    B::apply_impulse(world, entity, downhill_impulse);
                }
            }
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
    let entities: Vec<(Entity, ControllerConfig, MovementIntent, CharacterController)> = world
        .query::<(
            Entity,
            &ControllerConfig,
            &MovementIntent,
            &CharacterController,
        )>()
        .iter(world)
        .map(|(e, config, intent, controller)| (e, *config, intent.clone(), controller.clone()))
        .collect();

    let dt = B::get_fixed_timestep(world);

    for (entity, config, intent, controller) in entities {
        let is_grounded = controller.is_grounded(&config);

        // Flying downwards is disabled while grounded
        let fly_direction = intent.fly;
        let should_apply_fly = intent.is_flying() && !(is_grounded && intent.is_flying_down());

        if !should_apply_fly {
            continue;
        }

        let current_velocity = B::get_velocity(world, entity);
        let mass = B::get_mass(world, entity);
        let up = controller.ideal_up();

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

        // Record upward propulsion when actively flying up
        if fly_direction > 0.0 && change > 0.0 {
            if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
                controller.record_upward_propulsion(config.jump_spring_filter_duration);
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
    let entities: Vec<(Entity, ControllerConfig, MovementIntent, CharacterController)> = world
        .query::<(
            Entity,
            &ControllerConfig,
            &MovementIntent,
            &CharacterController,
        )>()
        .iter(world)
        .map(|(e, config, intent, controller)| (e, *config, intent.clone(), controller.clone()))
        .collect();

    // Get fixed timestep delta
    let dt = B::get_fixed_timestep(world);

    for (entity, config, intent, controller) in entities {
        let current_velocity = B::get_velocity(world, entity);
        let mass = B::get_mass(world, entity);
        let up = controller.ideal_up();
        let right = controller.ideal_right();

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
            // Clamp the slope tangent to respect max_slope_angle
            let slope_tangent = if controller.slope_angle <= config.max_slope_angle {
                // Within max slope angle, use actual slope tangent
                controller.ground_tangent()
            } else {
                // Slope exceeds max angle, clamp the tangent direction
                let up = controller.ideal_up();
                let ground_normal = controller.ground_normal();

                // Determine slope tilt direction based on which way the normal leans
                // If normal points away from right (negative dot), slope goes up when moving right
                let slope_tilt_sign = if ground_normal.dot(right) <= 0.0 {
                    1.0
                } else {
                    -1.0
                };

                // Rotate ideal_right by max_slope_angle in the tilt direction
                let cos_a = config.max_slope_angle.cos();
                let sin_a = config.max_slope_angle.sin() * slope_tilt_sign;

                // Compute clamped tangent: right rotated by max_slope_angle in the (right, up) plane
                Vec2::new(
                    right.x * cos_a + up.x * sin_a,
                    right.y * cos_a + up.y * sin_a,
                )
            };

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
            // Only the walking impulse is rotated - external velocity and spring system are unaffected
            let walk_impulse = slope_tangent * slope_velocity_delta * mass;
            B::apply_impulse(world, entity, walk_impulse);
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

            // Record upward propulsion when actively flying up
            if fly_direction > 0.0 && change > 0.0 {
                if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
                    controller.record_upward_propulsion(config.jump_spring_filter_duration);
                }
            }
        }
    }
}

/// Apply jump impulse when requested.
///
/// Jumping requires being grounded, touching a wall (with wall jumping enabled),
/// or within coyote time (which covers both ground and wall contact).
/// Jump requests are consumed directly from MovementIntent.
///
/// Jump direction depends on the jump type stored in last_jump_type:
/// - Ground: Jump along ground normal (or ideal up if using coyote time)
/// - LeftWall: Jump diagonally up-right at configured angle
/// - RightWall: Jump diagonally up-left at configured angle
///
/// Note: Expired jump requests are removed by `expire_jump_requests` before
/// this system runs, so we just check if a request exists.
pub fn apply_jump<B: CharacterPhysicsBackend>(world: &mut World) {
    use crate::config::JumpType;

    // Collect entities with pending jump requests that can jump
    // Expired requests are already removed by expire_jump_requests
    let entities: Vec<(Entity, ControllerConfig, CharacterController)> = world
        .query::<(
            Entity,
            &ControllerConfig,
            &CharacterController,
            &MovementIntent,
        )>()
        .iter(world)
        .filter_map(|(e, config, controller, intent)| {
            // Check if there's a valid jump request
            if intent.jump_request.is_none() {
                return None;
            }

            // Can jump if: grounded, touching wall (with wall jumping), or within coyote time
            let has_contact = controller.is_grounded(config)
                || (config.wall_jumping && controller.touching_wall());
            let can_jump = has_contact || controller.in_coyote_time();

            // For wall jumps via coyote time, also check that last_jump_type is valid
            let valid_jump_type = match controller.last_jump_type {
                JumpType::Ground => true,
                JumpType::LeftWall | JumpType::RightWall => config.wall_jumping,
            };

            if can_jump && valid_jump_type {
                Some((e, *config, controller.clone()))
            } else {
                None
            }
        })
        .collect();

    for (entity, config, controller) in entities {
        // Consume the jump request by taking it from MovementIntent
        if let Some(mut intent) = world.get_mut::<MovementIntent>(entity) {
            intent.take_jump_request();
        }

        // Calculate jump direction based on jump type
        let jump_direction = match controller.last_jump_type {
            JumpType::Ground => {
                // Normal ground jump: use ground normal or ideal up
                if controller.ground_detected() {
                    controller.ground_normal()
                } else {
                    controller.ideal_up()
                }
            }
            JumpType::LeftWall => {
                // Wall jump from left wall: jump up-right (away from wall)
                // Rotate ideal_up by wall_jump_angle clockwise (toward right)
                let up = controller.ideal_up();
                let angle = -config.wall_jump_angle; // Negative to rotate clockwise
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                Vec2::new(
                    up.x * cos_a - up.y * sin_a,
                    up.x * sin_a + up.y * cos_a,
                )
                .normalize()
            }
            JumpType::RightWall => {
                // Wall jump from right wall: jump up-left (away from wall)
                // Rotate ideal_up by wall_jump_angle counter-clockwise (toward left)
                let up = controller.ideal_up();
                let angle = config.wall_jump_angle; // Positive to rotate counter-clockwise
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                Vec2::new(
                    up.x * cos_a - up.y * sin_a,
                    up.x * sin_a + up.y * cos_a,
                )
                .normalize()
            }
        };

        // Apply jump impulse
        // jump_speed is the desired velocity change. Impulse = mass * delta_v
        // Scale by actual mass so velocity change equals jump_speed regardless of body mass.
        let mass = B::get_mass(world, entity);
        let impulse = jump_direction * config.jump_speed * mass;
        B::apply_impulse(world, entity, impulse);

        // Record upward propulsion for spring force filtering and fall gravity tracking
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.record_upward_propulsion(config.jump_spring_filter_duration);
            // Record jump for fall gravity system - tracks when we can cancel the jump
            controller.record_jump(config.jump_cancel_window);
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
    let entities: Vec<(Entity, ControllerConfig, CharacterController)> = world
        .query::<(Entity, &CharacterController, &ControllerConfig)>()
        .iter(world)
        .filter(|(_, _, config)| config.upright_torque_enabled)
        .map(|(e, controller, config)| (e, *config, controller.clone()))
        .collect();

    for (entity, config, controller) in entities {
        let current_rotation = B::get_rotation(world, entity);
        let angular_velocity = B::get_angular_velocity(world, entity);

        // Target angle: if not specified, use ideal_up angle minus PI/2 to get the body rotation
        let target_angle = config
            .upright_target_angle
            .unwrap_or_else(|| controller.ideal_up_angle() - consts::FRAC_PI_2);

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
