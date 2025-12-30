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

/// Process jump state: detect rising edge and create JumpRequests.
///
/// This system detects when `jump_pressed` transitions from false to true
/// and creates a new JumpRequest with the configured `jump_buffer_time`.
/// It also updates `jump_pressed_prev` for the next frame's edge detection.
///
/// Note: This system does not process raw input (keyboard, gamepad, etc.).
/// User code sets `jump_pressed` to a boolean, and this system handles
/// the rest (edge detection, buffering, etc.).
///
/// This must run before `tick_jump_request_timers` so that new requests
/// are created before timers are ticked.
pub fn process_jump_state(mut query: Query<(&mut MovementIntent, &ControllerConfig)>) {
    for (mut intent, config) in &mut query {
        // Detect rising edge: pressed this frame but not last frame
        if intent.jump_pressed && !intent.jump_pressed_prev {
            // Create a new jump request with the configured buffer time
            intent.request_jump(config.jump_buffer_time);
        }

        // Update previous state for next frame's edge detection
        intent.jump_pressed_prev = intent.jump_pressed;
    }
}

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
        // BUT not if we recently jumped (prevents coyote time from being granted after a jump)
        if (is_grounded || has_wall_contact) && !controller.recently_jumped() {
            controller.reset_coyote_timer(config.coyote_time);
        } else {
            controller.tick_coyote_timer(delta);
        }

        // Tick the jump spring filter timer
        controller.jump_spring_filter_timer.tick(delta);

        // Tick the jumped timer (for fall gravity tracking)
        controller.jumped_timer.tick(delta);

        // Tick the fall gravity timer
        controller.fall_gravity_timer.tick(delta);

        // Tick the wall jump movement block timer
        controller.wall_jump_movement_block_timer.tick(delta);

        // Tick the recently jumped timer
        controller.recently_jumped_timer.tick(delta);

        // Tick the jump max ascent timer
        controller.jump_max_ascent_timer.tick(delta);
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
pub fn update_jump_type(mut query: Query<(&mut CharacterController, &ControllerConfig)>) {
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
        let has_contact =
            controller.is_grounded(config) || (config.wall_jumping && controller.touching_wall());
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
        let max_range = target_height + config.grounding_distance;

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
        let height_above_target = -displacement;
        if displacement < 0.0
            && height_above_target <= config.grounding_distance
            && spring_force < 0.0
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
                    + config.spring_strength * config.grounding_distance * mass
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
                        + config.spring_strength * config.grounding_distance * mass
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

/// Apply fall gravity for early jump cancellation.
///
/// This system enables players to cancel jumps early by releasing the jump button,
/// making jumps feel less floaty. Fall gravity is triggered when:
///
/// 1. We jumped recently (within `jump_cancel_window`)
/// 2. AND either:
///    - Jump button is not held (player let go)
///    - OR we're moving downward (crossed the zenith)
///
/// When triggered, fall gravity is applied for `fall_gravity_duration`,
/// multiplying gravity by `fall_gravity`.
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
            // Only process airborne entities with fall_gravity > 1.0
            !controller.is_grounded(config) && config.fall_gravity > 1.0
        })
        .map(|(e, config, controller, intent)| {
            // Check if jump button is pressed (player wants full jump height)
            let jump_pressed = intent.jump_pressed;
            (e, *config, controller.clone(), jump_pressed)
        })
        .collect();

    for (entity, config, controller, jump_pressed) in entities {
        let up = controller.ideal_up();
        let velocity = B::get_velocity(world, entity);
        let vertical_velocity = velocity.dot(up);

        // Check if we should trigger fall gravity
        // Conditions:
        // 1. We jumped recently (within jump_cancel_window)
        // 2. AND we're past the recently_jumped grace period (protects against velocity flicker)
        // 3. AND either:
        //    - Jump button not pressed (player let go)
        //    - OR we're moving downward (crossed the zenith)
        //    - OR max ascent timer has expired (forces fall gravity after max jump duration)
        let max_ascent_expired = controller.jump_max_ascent_expired();
        let should_trigger = controller.in_jump_cancel_window()
            && !controller.recently_jumped()
            && (!jump_pressed || vertical_velocity < 0.0 || max_ascent_expired);

        // Trigger fall gravity if conditions are met
        if should_trigger && !controller.fall_gravity_active() {
            if let Some(mut ctrl) = world.get_mut::<CharacterController>(entity) {
                ctrl.trigger_fall_gravity(config.fall_gravity_duration);
            }
        }

        // Apply fall gravity if the timer is active
        // Re-check because we may have just triggered it
        let is_active = if let Some(ctrl) = world.get::<CharacterController>(entity) {
            ctrl.fall_gravity_active()
        } else {
            false
        };

        if is_active {
            // Apply fall gravity impulse
            // The regular gravity system applies: gravity * mass * dt
            // We want to add: gravity * mass * dt * (fall_gravity - 1)
            // This way total gravity becomes: gravity * mass * dt * fall_gravity
            let mass = B::get_mass(world, entity);
            let fall_multiplier = config.fall_gravity - 1.0;
            let fall_gravity_impulse = controller.gravity * mass * dt * fall_multiplier;
            B::apply_impulse(world, entity, fall_gravity_impulse);
        }
    }
}

/// Apply wall clinging dampening to slow descent when clinging to a wall.
///
/// When the character is touching a wall and has movement intent toward that wall,
/// this system applies an impulse to counteract downward motion along the wall surface.
/// This creates a "clinging" effect where the character slides down more slowly.
///
/// Dampening is NOT applied when:
/// - The character has downward fly intent (allowing intentional descent)
/// - The character is not moving toward the wall
/// - `wall_clinging_dampening` is 0.0
pub fn apply_wall_clinging_dampening<B: CharacterPhysicsBackend>(world: &mut World) {
    // Collect entities that might need wall dampening
    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterController,
        MovementIntent,
    )> = world
        .query::<(
            Entity,
            &ControllerConfig,
            &CharacterController,
            &MovementIntent,
        )>()
        .iter(world)
        .filter(|(_, config, controller, _)| {
            // Only process entities with wall clinging enabled and non-zero dampening
            config.wall_clinging
                && config.wall_clinging_dampening > 0.0
                && controller.touching_wall()
        })
        .map(|(e, config, controller, intent)| (e, *config, controller.clone(), intent.clone()))
        .collect();

    for (entity, config, controller, intent) in entities {
        let effective_walk = intent.effective_walk();

        // Determine which wall we're clinging to based on movement intent
        // We only apply dampening when moving TOWARD the wall
        let (wall_data, moving_toward_wall) =
            if effective_walk > 0.0 && controller.touching_right_wall() {
                // Moving right and touching right wall
                (controller.right_wall.as_ref(), true)
            } else if effective_walk < 0.0 && controller.touching_left_wall() {
                // Moving left and touching left wall
                (controller.left_wall.as_ref(), true)
            } else {
                (None, false)
            };

        // Skip if not moving toward a wall
        if !moving_toward_wall {
            continue;
        }

        let Some(wall) = wall_data else {
            continue;
        };

        // Skip if player has downward fly intent (wants to descend intentionally)
        if intent.is_flying_down() {
            continue;
        }

        // Calculate the "downward along wall" direction
        // This is perpendicular to the wall normal, aligned with gravity direction
        let wall_normal = wall.normal;
        let ideal_down = controller.ideal_down();

        // Wall tangent: perpendicular to normal. Two options: rotate 90° CW or CCW
        // We pick the one that has a component in the downward direction
        let tangent_option1 = Vec2::new(wall_normal.y, -wall_normal.x);
        let tangent_option2 = Vec2::new(-wall_normal.y, wall_normal.x);

        let wall_down = if tangent_option1.dot(ideal_down) > 0.0 {
            tangent_option1
        } else {
            tangent_option2
        };

        // Get current velocity
        let velocity = B::get_velocity(world, entity);
        let mass = B::get_mass(world, entity);

        // Get velocity component along the wall-down direction
        let down_velocity = velocity.dot(wall_down);

        // Calculate dampening factor
        let dampening_factor = config.wall_clinging_dampening;

        // Apply dampening based on movement direction along wall
        if down_velocity > 0.0 {
            // Moving downward along wall - always dampen
            // dampening of 1.0 = fully counteract, 0.0 = no effect
            // We apply a portion of the counteracting impulse each frame
            // to create smooth dampening rather than instant stop
            let velocity_reduction = down_velocity * dampening_factor;
            let dampening_impulse = -wall_down * velocity_reduction * mass;

            B::apply_impulse(world, entity, dampening_impulse);
        } else if down_velocity < 0.0 && config.wall_clinging_dampen_upward {
            // Moving upward along wall - only dampen if enabled
            // This creates a stickier wall cling effect
            let up_velocity = -down_velocity; // Make positive for calculation
            let velocity_reduction = up_velocity * dampening_factor;
            let dampening_impulse = wall_down * velocity_reduction * mass;

            B::apply_impulse(world, entity, dampening_impulse);
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
pub fn apply_walk<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(
        Entity,
        ControllerConfig,
        MovementIntent,
        CharacterController,
    )> = world
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
        let wall_cling_blocked = !config.wall_clinging
            && ((effective_walk > 0.0 && controller.touching_right_wall())
                || (effective_walk < 0.0 && controller.touching_left_wall()));

        // Check for wall jump movement blocking
        // blocked_direction > 0 means block rightward movement (positive walk)
        // blocked_direction < 0 means block leftward movement (negative walk)
        let blocked_direction = controller.get_wall_jump_blocked_direction();
        let wall_jump_blocked = blocked_direction != 0.0
            && ((blocked_direction > 0.0 && effective_walk > 0.0)
                || (blocked_direction < 0.0 && effective_walk < 0.0));

        let walk_blocked = wall_cling_blocked || wall_jump_blocked;

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

            // Only disable friction when walking forward (same direction as current velocity)
            let walking_forward = intent.is_walking()
                && (current_slope_speed.abs() <= 0.001
                    || effective_walk.signum() == current_slope_speed.signum());
            let friction_factor = if walking_forward {
                1.0
            } else {
                1.0 - config.friction
            };
            let new_slope_speed = (current_slope_speed + change) * friction_factor;
            let slope_velocity_delta = new_slope_speed - current_slope_speed;

            // Apply impulse along slope tangent: I = m * dv
            // Only the walking impulse is rotated - external velocity and spring system are unaffected
            let mut walk_impulse = slope_tangent * slope_velocity_delta * mass;

            // When we just jumped (within recently_jumped protection timer),
            // reject any upward component from the walking impulse.
            // This prevents walking up slopes and jumping from giving too strong jumps.
            if controller.recently_jumped() {
                let up = controller.ideal_up();
                let upward_component = walk_impulse.dot(up);
                if upward_component > 0.0 {
                    walk_impulse -= up * upward_component;
                }
            }

            B::apply_impulse(world, entity, walk_impulse);
        } else {
            // AIRBORNE: Use world-space horizontal axis
            let current_horizontal = current_velocity.dot(right);

            if intent.is_walking() {
                // Apply air control when actively walking
                // Calculate velocity change toward target, clamped by max acceleration
                let velocity_diff = desired_walk_speed - current_horizontal;
                let max_change = walk_accel * dt;
                let change = velocity_diff.clamp(-max_change, max_change);

                // Only disable air friction when walking forward (same direction as current velocity)
                let walking_forward = current_horizontal.abs() <= 0.001
                    || effective_walk.signum() == current_horizontal.signum();
                let friction_factor = if walking_forward {
                    1.0
                } else {
                    1.0 - config.air_friction
                };
                let new_horizontal = (current_horizontal + change) * friction_factor;
                let horizontal_delta = new_horizontal - current_horizontal;

                // Apply impulse along right axis: I = m * dv
                let walk_impulse = right * horizontal_delta * mass;
                B::apply_impulse(world, entity, walk_impulse);
            } else if config.air_friction > 0.0 && current_horizontal.abs() > 0.001 {
                // Not walking - apply air friction to slow down horizontal movement
                let friction_factor = 1.0 - config.air_friction;
                let new_horizontal = current_horizontal * friction_factor;
                let horizontal_delta = new_horizontal - current_horizontal;

                // Apply impulse along right axis: I = m * dv
                let friction_impulse = right * horizontal_delta * mass;
                B::apply_impulse(world, entity, friction_impulse);
            }
        }
    }
}

/// Apply flying propulsion (vertical and horizontal) based on intent.
///
/// Handles flying movement using impulses scaled by mass. Uses the dedicated
/// flying configuration (`fly_max_speed`, `fly_vertical_speed_ratio`,
/// `fly_gravity_compensation`) for speed and acceleration.
///
/// Key behaviors:
/// - Upward propulsion is boosted by gravity based on `fly_gravity_compensation`
/// - Vertical speed is scaled by `fly_vertical_speed_ratio`
/// - When grounded: applies friction to horizontal flying like walking
/// - When airborne: horizontal flying uses full fly_max_speed (no air control reduction)
/// - Flying downwards: stops propulsion at max speed but does not counteract gravity
/// - Flying downwards is disabled while grounded (only horizontal and up work grounded)
pub fn apply_fly<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(
        Entity,
        ControllerConfig,
        MovementIntent,
        CharacterController,
    )> = world
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
        let current_velocity = B::get_velocity(world, entity);
        let mass = B::get_mass(world, entity);
        let up = controller.ideal_up();
        let right = controller.ideal_right();

        // === VERTICAL FLYING ===
        // Flying downwards is disabled while grounded
        let fly_direction = intent.fly;
        let should_apply_vertical_fly =
            intent.is_flying() && !(is_grounded && intent.is_flying_down());

        if should_apply_vertical_fly {
            let current_vertical = current_velocity.dot(up);

            // Apply vertical speed ratio to the target speed
            let vertical_max_speed = config.fly_max_speed * config.fly_vertical_speed_ratio;
            let desired_vertical = intent.effective_fly() * vertical_max_speed;

            // When flying down: stop propulsion at max speed but don't counteract gravity
            // This means we only apply force if we're slower than max speed going down
            let should_apply_downward =
                !intent.is_flying_down() || current_vertical > -vertical_max_speed;

            if should_apply_downward {
                // Use fly_acceleration with vertical ratio as base
                let mut fly_accel =
                    config.fly_acceleration * config.fly_vertical_acceleration_ratio;

                // Boost upward propulsion by gravity based on compensation setting
                if fly_direction > 0.0 {
                    fly_accel += controller.gravity.length() * config.fly_gravity_compensation;
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

        // === HORIZONTAL FLYING ===
        if intent.is_flying_horizontal() {
            let current_horizontal = current_velocity.dot(right);
            let desired_horizontal = intent.effective_fly_horizontal() * config.fly_max_speed;

            // Use fly_acceleration for horizontal flying
            let fly_accel = config.fly_acceleration;

            // Calculate velocity change toward target, clamped by max acceleration
            let velocity_diff = desired_horizontal - current_horizontal;
            let max_change = fly_accel * dt;
            let change = velocity_diff.clamp(-max_change, max_change);

            // Only disable friction when flying forward (same direction as current velocity)
            let flying_forward = current_horizontal.abs() <= 0.001
                || intent.effective_fly_horizontal().signum() == current_horizontal.signum();
            let friction_factor = if flying_forward {
                1.0
            } else if is_grounded {
                1.0 - config.friction
            } else {
                1.0 - config.air_friction
            };

            let new_horizontal = (current_horizontal + change) * friction_factor;
            let horizontal_delta = new_horizontal - current_horizontal;

            // Apply impulse along right axis: I = m * dv
            let fly_horizontal_impulse = right * horizontal_delta * mass;
            B::apply_impulse(world, entity, fly_horizontal_impulse);
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
/// - Ground: Jump along ideal up (not affected by slope)
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
                // Ground jump: always use ideal up, not affected by slope
                controller.ideal_up()
            }
            JumpType::LeftWall => {
                // Wall jump from left wall: jump up-right (away from wall)
                let up = controller.ideal_up();
                if config.wall_jump_retain_height {
                    // Retain height mode: same vertical as ground jump, add horizontal for angle
                    // The horizontal component is added "for free" on top of the full upward impulse
                    let horizontal = config.wall_jump_angle.tan();
                    let right = Vec2::new(up.y, -up.x); // 90° clockwise from up
                    up + right * horizontal
                } else {
                    // Classic rotation mode: rotate ideal_up by wall_jump_angle clockwise
                    let angle = -config.wall_jump_angle; // Negative to rotate clockwise
                    let cos_a = angle.cos();
                    let sin_a = angle.sin();
                    Vec2::new(up.x * cos_a - up.y * sin_a, up.x * sin_a + up.y * cos_a).normalize()
                }
            }
            JumpType::RightWall => {
                // Wall jump from right wall: jump up-left (away from wall)
                let up = controller.ideal_up();
                if config.wall_jump_retain_height {
                    // Retain height mode: same vertical as ground jump, add horizontal for angle
                    // The horizontal component is added "for free" on top of the full upward impulse
                    let horizontal = config.wall_jump_angle.tan();
                    let right = Vec2::new(up.y, -up.x); // 90° clockwise from up
                    up - right * horizontal // Subtract right = add left
                } else {
                    // Classic rotation mode: rotate ideal_up by wall_jump_angle counter-clockwise
                    let angle = config.wall_jump_angle; // Positive to rotate counter-clockwise
                    let cos_a = angle.cos();
                    let sin_a = angle.sin();
                    Vec2::new(up.x * cos_a - up.y * sin_a, up.x * sin_a + up.y * cos_a).normalize()
                }
            }
        };

        // Compensate for downward velocity before applying jump impulse
        // This helps the character jump with consistent height regardless of falling speed
        let mass = B::get_mass(world, entity);
        let velocity = B::get_velocity(world, entity);
        let ideal_up = controller.ideal_up();
        let vertical_velocity = velocity.dot(ideal_up);

        // Only compensate if moving downward (negative vertical velocity)
        if vertical_velocity < 0.0 {
            // Ground jumps: fully compensate (cancel all downward velocity)
            // Wall jumps: compensate based on config (0.0 = none, 1.0 = full)
            let compensation = match controller.last_jump_type {
                JumpType::Ground => 1.0,
                JumpType::LeftWall | JumpType::RightWall => config.wall_jump_velocity_compensation,
            };

            if compensation > 0.0 {
                // Apply impulse to cancel downward velocity (impulse = mass * delta_v)
                let compensation_impulse = ideal_up * (-vertical_velocity * compensation * mass);
                B::apply_impulse(world, entity, compensation_impulse);
            }
        }

        // Apply jump impulse, reducing it by pre-existing upward velocity
        // This prevents "super jumps" when jumping while already moving upward
        // jump_speed is the desired velocity change. Impulse = mass * delta_v
        // Scale by actual mass so velocity change equals jump_speed regardless of body mass.
        let effective_jump_speed = if vertical_velocity > 0.0 {
            // Reduce jump speed by current upward velocity, scaled by compensation factor
            let reduction = vertical_velocity * config.jump_upward_velocity_compensation;
            (config.jump_speed - reduction).max(0.0)
        } else {
            config.jump_speed
        };
        let impulse = jump_direction * effective_jump_speed * mass;
        B::apply_impulse(world, entity, impulse);

        // Record upward propulsion for spring force filtering and fall gravity tracking
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.record_upward_propulsion(config.jump_spring_filter_duration);
            // Record jump for fall gravity system - tracks when we can cancel the jump
            controller.record_jump(config.jump_cancel_window);
            // Record recently jumped for fall gravity protection and coyote rejection
            controller.record_recently_jumped(config.recently_jumped_duration);
            // Record jump max ascent for forcing fall gravity after max duration
            controller.record_jump_max_ascent(config.jump_max_ascent_duration);

            // For wall jumps, block movement toward the wall to help jump away correctly
            // LeftWall jump: block leftward movement (toward the left wall)
            // RightWall jump: block rightward movement (toward the right wall)
            match controller.last_jump_type {
                JumpType::LeftWall => {
                    controller.record_wall_jump_movement_block(
                        config.wall_jump_movement_block_duration,
                        -1.0, // Block leftward movement
                    );
                }
                JumpType::RightWall => {
                    controller.record_wall_jump_movement_block(
                        config.wall_jump_movement_block_duration,
                        1.0, // Block rightward movement
                    );
                }
                JumpType::Ground => {}
            }
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
