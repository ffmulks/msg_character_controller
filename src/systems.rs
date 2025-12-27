//! Core controller systems.
//!
//! These systems implement the floating character controller behavior.
//! They are generic over the physics backend to allow different physics
//! engines to be used.

use std::f32::consts;

use bevy::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, CharacterOrientation, ControllerConfig, StairConfig};
use crate::detection::{GroundInfo, SensorCast, WallInfo};
use crate::intent::{FlyIntent, JumpRequest, WalkIntent};
use crate::state::{Airborne, Grounded, TouchingWall};

/// Update ground detection for all character controllers.
///
/// Performs a shapecast downward to detect ground and compute:
/// - Distance to ground
/// - Ground normal (for slopes) - from the shapecast hit
/// - Slope angle
/// - Step detection (for stairs)
pub fn update_ground_detection<B: CharacterPhysicsBackend>(world: &mut World) {
    // Collect entities that need ground detection
    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterOrientation,
        Option<StairConfig>,
    )> = world
        .query::<(
            Entity,
            &CharacterController,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            Option<&StairConfig>,
        )>()
        .iter(world)
        .map(|(e, _, config, orientation, stair)| {
            (
                e,
                *config,
                orientation.copied().unwrap_or_default(),
                stair.copied(),
            )
        })
        .collect();

    for (entity, config, orientation, stair_config) in entities {
        let position = B::get_position(world, entity);
        let velocity = B::get_velocity(world, entity);

        // Use orientation to determine "down" direction
        let down = orientation.down();

        // Perform ground shapecast - single wide cast replaces multiple raycasts
        // The shapecast normal provides slope information directly
        let ground_cast = B::shapecast(
            world,
            position,
            down,
            config.ground_cast_length,
            config.ground_cast_width,
            entity,
        );

        // Compute ground info from shapecast
        let mut ground_info = compute_ground_info(&ground_cast, &orientation, &config);

        // Check for step if enabled and we hit a wall-like surface
        if let Some(stair) = stair_config {
            if stair.enabled && !ground_info.is_walkable && ground_cast.hit {
                let step_result =
                    check_for_step::<B>(world, entity, position, velocity, &orientation, &stair);

                if step_result.hit {
                    ground_info.step_detected = true;
                    ground_info.step_height = step_result.distance;
                }
            }
        }

        // Update time since grounded
        let dt = B::get_fixed_timestep(world);
        if let Some(existing) = world.get::<GroundInfo>(entity) {
            if ground_info.is_grounded(config.float_height, config.cling_distance) {
                ground_info.time_since_grounded = 0.0;
            } else {
                ground_info.time_since_grounded = existing.time_since_grounded + dt;
            }
        }

        // Insert or update ground info
        if let Some(mut existing) = world.get_mut::<GroundInfo>(entity) {
            *existing = ground_info;
        } else {
            world.entity_mut(entity).insert(ground_info);
        }
    }
}

/// Compute ground info from a single shapecast.
/// The shapecast's normal provides slope information directly.
fn compute_ground_info(
    cast: &SensorCast,
    orientation: &CharacterOrientation,
    config: &ControllerConfig,
) -> GroundInfo {
    if !cast.hit {
        return GroundInfo::default();
    }

    let normal = cast.normal;

    // Compute slope angle (angle between normal and up direction)
    let up = orientation.up();
    let dot = normal.dot(up).clamp(-1.0, 1.0);
    let slope_angle = dot.acos();

    // Check if walkable
    let is_walkable = slope_angle <= config.max_slope_angle;

    GroundInfo {
        detected: true,
        distance: cast.distance,
        normal,
        contact_point: cast.point,
        slope_angle,
        is_walkable,
        step_detected: false,
        step_height: 0.0,
        time_since_grounded: 0.0,
        ground_entity: cast.entity,
    }
}

/// Check for a climbable step in front of the character.
fn check_for_step<B: CharacterPhysicsBackend>(
    world: &World,
    entity: Entity,
    position: Vec2,
    velocity: Vec2,
    orientation: &CharacterOrientation,
    config: &StairConfig,
) -> SensorCast {
    let down = orientation.down();
    let right = orientation.right();

    // Project velocity onto horizontal axis and determine movement direction
    let horizontal_vel = velocity.dot(right);
    let move_dir = if horizontal_vel.abs() > 0.1 {
        right * horizontal_vel.signum()
    } else {
        return SensorCast::miss();
    };

    // Cast forward at step height to find the top of potential step
    let step_origin = position - down * config.max_step_height;
    let forward_cast = B::raycast(
        world,
        step_origin,
        move_dir,
        config.step_check_distance,
        entity,
    );

    // If we didn't hit anything forward at step height, check downward
    if !forward_cast.hit {
        let check_origin = step_origin + move_dir * config.step_check_distance;
        let down_cast = B::raycast(
            world,
            check_origin,
            down,
            config.max_step_height + 2.0,
            entity,
        );

        // If we found ground and it's higher than current ground, it's a step
        if down_cast.hit && down_cast.distance < config.max_step_height {
            return SensorCast::hit(
                config.max_step_height - down_cast.distance,
                down_cast.normal,
                down_cast.point,
                down_cast.entity,
            );
        }
    }

    SensorCast::miss()
}

/// Update wall detection for all character controllers.
pub fn update_wall_detection<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation)> = world
        .query::<(
            Entity,
            &CharacterController,
            &ControllerConfig,
            Option<&CharacterOrientation>,
        )>()
        .iter(world)
        .map(|(e, _, config, orientation)| (e, *config, orientation.copied().unwrap_or_default()))
        .collect();

    for (entity, config, orientation) in entities {
        let position = B::get_position(world, entity);
        let left = orientation.left();
        let right = orientation.right();

        // Shapecast left and right for wall detection (in local orientation)
        let left_cast = B::shapecast(
            world,
            position,
            left,
            config.wall_cast_length,
            config.wall_cast_width,
            entity,
        );

        let right_cast = B::shapecast(
            world,
            position,
            right,
            config.wall_cast_length,
            config.wall_cast_width,
            entity,
        );

        let wall_info = WallInfo {
            left_detected: left_cast.hit,
            left_distance: left_cast.distance,
            left_normal: left_cast.normal,
            right_detected: right_cast.hit,
            right_distance: right_cast.distance,
            right_normal: right_cast.normal,
        };

        // Insert or update wall info
        if let Some(mut existing) = world.get_mut::<WallInfo>(entity) {
            *existing = wall_info;
        } else {
            world.entity_mut(entity).insert(wall_info);
        }
    }
}

/// Apply the floating spring force to maintain float height.
pub fn apply_floating_spring<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(Entity, ControllerConfig, CharacterOrientation, GroundInfo)> = world
        .query::<(
            Entity,
            &CharacterController,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            &GroundInfo,
        )>()
        .iter(world)
        .filter(|(_, controller, _, _, _)| controller.is_walking())
        .map(|(e, _, config, orientation, ground)| {
            (
                e,
                *config,
                orientation.copied().unwrap_or_default(),
                *ground,
            )
        })
        .collect();

    for (entity, config, orientation, ground) in entities {
        if !ground.detected {
            continue;
        }

        let velocity = B::get_velocity(world, entity);
        let gravity = B::get_gravity(world, entity);
        let up = orientation.up();

        // Calculate height error
        let height_error = ground.height_error(config.float_height);

        // Spring force: F = k * x - c * v
        // Where x is height error and v is vertical velocity (along up direction)
        let vertical_velocity = velocity.dot(up);
        let spring_force =
            config.spring_strength * height_error - config.spring_damping * vertical_velocity;

        // Apply force in the "up" direction
        let force = up * spring_force;
        B::apply_force(world, entity, force);

        // Apply cling force when above float height but within cling distance
        // This helps the character stick to ground on bumpy terrain
        if height_error < 0.0
            && ground.distance <= config.float_height + config.cling_distance
            && config.cling_strength > 0.0
        {
            let cling_force = gravity * config.cling_strength;
            B::apply_force(world, entity, cling_force);
        }

        // Apply extra gravity when going uphill
        if ground.is_on_slope() && ground.slope_angle > 0.0 {
            let extra_gravity = gravity * (config.uphill_gravity_multiplier - 1.0);
            B::apply_force(world, entity, extra_gravity);
        }
    }
}

/// Apply walking movement based on intent.
pub fn apply_walk_movement<B: CharacterPhysicsBackend>(world: &mut World) {
    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterOrientation,
        WalkIntent,
        GroundInfo,
    )> = world
        .query::<(
            Entity,
            &CharacterController,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            &WalkIntent,
            &GroundInfo,
        )>()
        .iter(world)
        .filter(|(_, controller, _, _, _, _)| controller.is_walking())
        .map(|(e, _, config, orientation, intent, ground)| {
            (
                e,
                *config,
                orientation.copied().unwrap_or_default(),
                *intent,
                *ground,
            )
        })
        .collect();

    // Get fixed timestep delta, with fallback for testing scenarios where delta might be 0
    let dt = world
        .get_resource::<Time<Fixed>>()
        .map(|t| t.delta_secs())
        .filter(|&d| d > 0.0)
        .unwrap_or(1.0 / 60.0);

    for (entity, config, orientation, intent, ground) in entities {
        let current_velocity = B::get_velocity(world, entity);
        let up = orientation.up();

        // Determine movement direction based on ground normal (tangent to slope)
        // or use the character's local right direction if not grounded
        let right = if ground.detected {
            ground.tangent()
        } else {
            orientation.right()
        };

        // Calculate desired velocity
        let desired_speed = intent.effective() * config.max_speed;

        // Determine acceleration based on grounded state
        let is_grounded = ground.is_grounded(config.float_height, config.cling_distance);
        let accel = if is_grounded {
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
        let friction_multiplier = if !intent.is_active() && is_grounded {
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

    // Get fixed timestep delta, with fallback for testing scenarios where delta might be 0
    let dt = world
        .get_resource::<Time<Fixed>>()
        .map(|t| t.delta_secs())
        .filter(|&d| d > 0.0)
        .unwrap_or(1.0 / 60.0);

    for (entity, config, orientation, intent) in entities {
        let current_velocity = B::get_velocity(world, entity);

        // Convert local intent to world velocity
        // intent.x = right/left, intent.y = up/down
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
pub fn apply_jump<B: CharacterPhysicsBackend>(world: &mut World) {
    let time = world
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0);

    let entities: Vec<(
        Entity,
        ControllerConfig,
        CharacterOrientation,
        GroundInfo,
        bool,
    )> = world
        .query::<(
            Entity,
            &CharacterController,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            &GroundInfo,
            &JumpRequest,
        )>()
        .iter(world)
        .filter(|(_, controller, _, _, _, _)| controller.is_walking())
        .map(|(e, _, config, orientation, ground, jump)| {
            let can_jump = jump.is_valid(time, config.jump_buffer_time)
                && (ground.is_grounded(config.float_height, config.cling_distance)
                    || ground.time_since_grounded < config.coyote_time);
            (
                e,
                *config,
                orientation.copied().unwrap_or_default(),
                *ground,
                can_jump,
            )
        })
        .collect();

    for (entity, config, orientation, ground, can_jump) in entities {
        if !can_jump {
            continue;
        }

        // Consume the jump request
        if let Some(mut jump) = world.get_mut::<JumpRequest>(entity) {
            jump.consume();
        }

        // Calculate jump direction: use ground normal if detected, otherwise character's up
        let up = if ground.detected {
            ground.normal
        } else {
            orientation.up()
        };

        // Apply jump impulse
        let impulse = up * config.jump_speed;
        B::apply_impulse(world, entity, impulse);
    }
}

/// Sync state marker components based on detection results.
pub fn sync_state_markers(
    mut commands: Commands,
    q_controllers: Query<(
        Entity,
        &CharacterController,
        &ControllerConfig,
        Option<&CharacterOrientation>,
        &GroundInfo,
        Option<&WallInfo>,
        Has<Grounded>,
        Has<Airborne>,
        Has<TouchingWall>,
    )>,
) {
    for (
        entity,
        controller,
        config,
        orientation_opt,
        ground,
        wall,
        has_grounded,
        has_airborne,
        has_wall,
    ) in &q_controllers
    {
        let orientation = orientation_opt.copied().unwrap_or_default();
        let is_grounded = controller.is_walking()
            && ground.is_grounded(config.float_height, config.cling_distance);

        // Sync Grounded/Airborne
        if is_grounded && !has_grounded {
            commands.entity(entity).insert(Grounded);
            commands.entity(entity).remove::<Airborne>();
        } else if !is_grounded && has_grounded {
            commands.entity(entity).remove::<Grounded>();
            commands.entity(entity).insert(Airborne);
        } else if !is_grounded && !has_airborne && !has_grounded {
            commands.entity(entity).insert(Airborne);
        }

        // Sync TouchingWall using character's local directions
        if let Some(wall) = wall {
            if wall.any_wall() && !has_wall {
                let (direction, normal) = if wall.left_detected {
                    (orientation.left(), wall.left_normal)
                } else {
                    (orientation.right(), wall.right_normal)
                };
                commands
                    .entity(entity)
                    .insert(TouchingWall::new(direction, normal));
            } else if !wall.any_wall() && has_wall {
                commands.entity(entity).remove::<TouchingWall>();
            }
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
        // The up direction defines where "up" is, so the target rotation
        // should align the character's local up with the orientation's up
        let target_angle = orientation.angle() - consts::FRAC_PI_2;

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
