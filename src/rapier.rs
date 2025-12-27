//! Rapier2D physics backend implementation.
//!
//! This module provides the physics backend for Bevy Rapier2D.
//! Enable with the `rapier2d` feature.

use bevy::prelude::*;
use bevy_rapier2d::parry::shape::Segment;
use bevy_rapier2d::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, CharacterOrientation, ControllerConfig, StairConfig};
use crate::detection::{GroundInfo, SensorCast, WallInfo};

/// Rapier2D physics backend for the character controller.
///
/// This backend uses `bevy_rapier2d` for all physics operations including
/// shapecasting, force application, and velocity manipulation.
///
/// # Note on Shapecasting
///
/// Due to Rapier2D's system parameter architecture, shapecasting is handled by
/// dedicated Rapier systems (`rapier_ground_detection`, `rapier_wall_detection`)
/// rather than through the generic backend trait. The `shapecast` method returns
/// a miss and should not be used directly - use the component-based detection
/// results instead.
///
/// # Usage
///
/// ```rust,no_run
/// use bevy::prelude::*;
/// use bevy_rapier2d::prelude::*;
/// use msg_character_controller::prelude::*;
///
/// App::new()
///     .add_plugins(DefaultPlugins)
///     .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
///     .add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default())
///     .run();
/// ```
pub struct Rapier2dBackend;

impl CharacterPhysicsBackend for Rapier2dBackend {
    type VelocityComponent = Velocity;

    fn plugin() -> impl Plugin {
        Rapier2dBackendPlugin
    }

    fn shapecast(
        _world: &World,
        _origin: Vec2,
        _direction: Vec2,
        _max_distance: f32,
        _shape_width: f32,
        _exclude_entity: Entity,
    ) -> SensorCast {
        // Shapecasting is handled by Rapier-specific systems that use ReadRapierContext.
        // This method is not used for the Rapier backend - detection is done via
        // rapier_ground_detection and rapier_wall_detection systems.
        SensorCast::miss()
    }

    fn raycast(
        _world: &World,
        _origin: Vec2,
        _direction: Vec2,
        _max_distance: f32,
        _exclude_entity: Entity,
    ) -> SensorCast {
        // Raycasting is handled by Rapier-specific systems.
        SensorCast::miss()
    }

    fn get_velocity(world: &World, entity: Entity) -> Vec2 {
        world
            .get::<Velocity>(entity)
            .map(|v| v.linvel)
            .unwrap_or(Vec2::ZERO)
    }

    fn set_velocity(world: &mut World, entity: Entity, velocity: Vec2) {
        if let Some(mut vel) = world.get_mut::<Velocity>(entity) {
            vel.linvel = velocity;
        }
    }

    fn apply_impulse(world: &mut World, entity: Entity, impulse: Vec2) {
        if let Some(mut ext_impulse) = world.get_mut::<ExternalImpulse>(entity) {
            ext_impulse.impulse += impulse;
        } else if let Some(mut vel) = world.get_mut::<Velocity>(entity) {
            // Fallback: apply as velocity change if no ExternalImpulse component
            vel.linvel += impulse;
        }
    }

    fn apply_force(world: &mut World, entity: Entity, force: Vec2) {
        if let Some(mut ext_force) = world.get_mut::<ExternalForce>(entity) {
            ext_force.force += force;
        }
    }

    fn apply_torque(world: &mut World, entity: Entity, torque: f32) {
        if let Some(mut ext_force) = world.get_mut::<ExternalForce>(entity) {
            ext_force.torque += torque;
        }
    }

    fn get_angular_velocity(world: &World, entity: Entity) -> f32 {
        world
            .get::<Velocity>(entity)
            .map(|v| v.angvel)
            .unwrap_or(0.0)
    }

    fn get_rotation(world: &World, entity: Entity) -> f32 {
        world
            .get::<Transform>(entity)
            .map(|t| {
                let (_, _, z) = t.rotation.to_euler(EulerRot::XYZ);
                z
            })
            .or_else(|| {
                world.get::<GlobalTransform>(entity).map(|t| {
                    let (_, rotation, _) = t.to_scale_rotation_translation();
                    let (_, _, z) = rotation.to_euler(EulerRot::XYZ);
                    z
                })
            })
            .unwrap_or(0.0)
    }

    fn get_gravity(world: &World, entity: Entity) -> Vec2 {
        // Check for per-entity gravity scale
        let gravity_scale = world
            .get::<GravityScale>(entity)
            .map(|gs| gs.0)
            .unwrap_or(1.0);

        // For Rapier, we store gravity in a dedicated component or use default
        // The actual gravity computation should be done by the game's gravity system
        // Here we return the scale factor times a default gravity
        Vec2::new(0.0, -9.81 * 16.0) * gravity_scale
    }

    fn get_position(world: &World, entity: Entity) -> Vec2 {
        world
            .get::<Transform>(entity)
            .map(|t| t.translation.xy())
            .or_else(|| {
                world
                    .get::<GlobalTransform>(entity)
                    .map(|t| t.translation().xy())
            })
            .unwrap_or(Vec2::ZERO)
    }

    fn get_fixed_timestep(world: &World) -> f32 {
        world
            .get_resource::<Time<Fixed>>()
            .map(|t| t.delta_secs())
            .filter(|&d| d > 0.0)
            .unwrap_or(1.0 / 60.0)
    }
}

/// Plugin that sets up Rapier2D-specific systems for the character controller.
pub struct Rapier2dBackendPlugin;

impl Plugin for Rapier2dBackendPlugin {
    fn build(&self, app: &mut App) {
        // Add Rapier-specific detection systems that use ReadRapierContext
        // These run BEFORE the generic controller systems
        app.add_systems(
            FixedUpdate,
            (rapier_ground_detection, rapier_wall_detection)
                .chain()
                .before(crate::systems::apply_floating_spring::<Rapier2dBackend>),
        );

        // Ensure external force/impulse are reset each frame
        app.add_systems(
            FixedPostUpdate,
            reset_external_forces.after(crate::systems::reset_jump_requests),
        );
    }
}

/// Rapier-specific ground detection system using shapecast.
fn rapier_ground_detection(
    rapier_context: ReadRapierContext,
    mut q_controllers: Query<
        (
            Entity,
            &GlobalTransform,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            Option<&StairConfig>,
            &Velocity,
            Option<&mut GroundInfo>,
        ),
        With<CharacterController>,
    >,
    mut commands: Commands,
    time: Res<Time<Fixed>>,
) {
    let Ok(context) = rapier_context.single() else {
        return;
    };

    let default_orientation = CharacterOrientation::default();

    for (entity, transform, config, orientation_opt, stair_config, velocity, ground_info_opt) in
        &mut q_controllers
    {
        let position = transform.translation().xy();

        // Get orientation (use default if component not present)
        let orientation = orientation_opt.unwrap_or(&default_orientation);
        let down = orientation.down();
        let right = orientation.right();

        // Build query filter
        let filter = QueryFilter::default()
            .exclude_rigid_body(entity)
            .exclude_sensors();

        // Create a flat horizontal segment for ground detection
        // The segment is perpendicular to the cast direction (horizontal in local space)
        let half_width = config.ground_cast_width / 2.0;
        let segment_a = right * -half_width;
        let segment_b = right * half_width;
        let shape = Segment::new(segment_a.into(), segment_b.into());

        // Compute rotation angle for the shape to align with character orientation
        let shape_rotation = orientation.angle() - std::f32::consts::FRAC_PI_2;

        // Perform shapecast - single wide cast replaces multiple raycasts
        let shape_result = context.cast_shape(
            position,
            shape_rotation,
            down,
            &shape,
            ShapeCastOptions {
                max_time_of_impact: config.ground_cast_length,
                stop_at_penetration: false,
                ..default()
            },
            filter,
        );

        // Compute ground info from shapecast
        let mut new_ground_info = if let Some((hit_entity, hit)) = shape_result {
            // Get normal from hit details (if available) or use character's up direction
            let normal = hit.details.map(|d| d.normal1).unwrap_or(orientation.up());
            let up = orientation.up();
            let dot = normal.dot(up).clamp(-1.0, 1.0);
            let slope_angle = dot.acos();
            let is_walkable = slope_angle <= config.max_slope_angle;

            GroundInfo {
                detected: true,
                distance: hit.time_of_impact,
                normal,
                contact_point: position + down * hit.time_of_impact,
                slope_angle,
                is_walkable,
                step_detected: false,
                step_height: 0.0,
                time_since_grounded: 0.0,
                ground_entity: Some(hit_entity),
            }
        } else {
            GroundInfo::default()
        };

        // Check for stairs if enabled
        if let Some(stair) = stair_config {
            if stair.enabled && !new_ground_info.is_walkable && shape_result.is_some() {
                if let Some(step_height) = check_stair_step(
                    &context,
                    entity,
                    position,
                    velocity.linvel,
                    orientation,
                    stair,
                ) {
                    new_ground_info.step_detected = true;
                    new_ground_info.step_height = step_height;
                }
            }
        }

        // Update time since grounded
        let dt = time.delta_secs();
        if let Some(mut existing) = ground_info_opt {
            if new_ground_info.is_grounded(config.float_height, config.cling_distance) {
                new_ground_info.time_since_grounded = 0.0;
            } else {
                new_ground_info.time_since_grounded = existing.time_since_grounded + dt;
            }
            *existing = new_ground_info;
        } else {
            commands.entity(entity).insert(new_ground_info);
        }
    }
}

fn check_stair_step(
    context: &RapierContext,
    entity: Entity,
    position: Vec2,
    velocity: Vec2,
    orientation: &CharacterOrientation,
    config: &StairConfig,
) -> Option<f32> {
    let down = orientation.down();
    let right = orientation.right();

    // Project velocity onto horizontal axis and determine movement direction
    let horizontal_vel = velocity.dot(right);
    let move_dir = if horizontal_vel.abs() > 0.1 {
        right * horizontal_vel.signum()
    } else {
        return None;
    };

    let filter = QueryFilter::default()
        .exclude_rigid_body(entity)
        .exclude_sensors();

    // Cast forward at step height (elevated in the "up" direction)
    let step_origin = position - down * config.max_step_height;
    let forward_hit = context.cast_ray(
        step_origin,
        move_dir,
        config.step_check_distance,
        true,
        filter,
    );

    // If no hit forward at step height, check down to find step top
    if forward_hit.is_none() {
        let check_origin = step_origin + move_dir * config.step_check_distance;
        if let Some((_, toi)) = context.cast_ray(
            check_origin,
            down,
            config.max_step_height + 2.0,
            true,
            filter,
        ) {
            if toi < config.max_step_height {
                return Some(config.max_step_height - toi);
            }
        }
    }

    None
}

/// Rapier-specific wall detection system using shapecast.
fn rapier_wall_detection(
    rapier_context: ReadRapierContext,
    mut q_controllers: Query<
        (
            Entity,
            &GlobalTransform,
            &ControllerConfig,
            Option<&CharacterOrientation>,
            Option<&mut WallInfo>,
        ),
        With<CharacterController>,
    >,
    mut commands: Commands,
) {
    let Ok(context) = rapier_context.single() else {
        return;
    };

    let default_orientation = CharacterOrientation::default();

    for (entity, transform, config, orientation_opt, wall_info_opt) in &mut q_controllers {
        let position = transform.translation().xy();

        // Get orientation (use default if component not present)
        let orientation = orientation_opt.unwrap_or(&default_orientation);
        let left = orientation.left();
        let right = orientation.right();
        let up = orientation.up();

        let filter = QueryFilter::default()
            .exclude_rigid_body(entity)
            .exclude_sensors();

        // Create a vertical segment for wall detection (aligned with up direction)
        let half_height = config.wall_cast_width / 2.0;
        let segment_a = up * -half_height;
        let segment_b = up * half_height;
        let shape = Segment::new(segment_a.into(), segment_b.into());

        // Compute rotation angle for the shape
        let shape_rotation = orientation.angle();

        let shape_opts = ShapeCastOptions {
            max_time_of_impact: config.wall_cast_length,
            stop_at_penetration: false,
            ..default()
        };

        // Shapecast left
        let left_hit =
            context.cast_shape(position, shape_rotation, left, &shape, shape_opts, filter);

        // Shapecast right
        let right_hit =
            context.cast_shape(position, shape_rotation, right, &shape, shape_opts, filter);

        let new_wall_info = WallInfo {
            left_detected: left_hit.is_some(),
            left_distance: left_hit
                .as_ref()
                .map(|(_, h)| h.time_of_impact)
                .unwrap_or(f32::MAX),
            left_normal: left_hit
                .as_ref()
                .and_then(|(_, h)| h.details.map(|d| d.normal1))
                .unwrap_or(right), // Default normal points away from wall
            right_detected: right_hit.is_some(),
            right_distance: right_hit
                .as_ref()
                .map(|(_, h)| h.time_of_impact)
                .unwrap_or(f32::MAX),
            right_normal: right_hit
                .as_ref()
                .and_then(|(_, h)| h.details.map(|d| d.normal1))
                .unwrap_or(left), // Default normal points away from wall
        };

        if let Some(mut existing) = wall_info_opt {
            *existing = new_wall_info;
        } else {
            commands.entity(entity).insert(new_wall_info);
        }
    }
}

/// Reset external forces after they've been applied.
fn reset_external_forces(mut q: Query<&mut ExternalForce, With<CharacterController>>) {
    for mut ext_force in &mut q {
        ext_force.force = Vec2::ZERO;
        ext_force.torque = 0.0;
    }
}

/// Bundle for creating a character with Rapier2D physics.
#[derive(Bundle, Default)]
pub struct Rapier2dCharacterBundle {
    pub rigid_body: RigidBody,
    pub velocity: Velocity,
    pub external_force: ExternalForce,
    pub external_impulse: ExternalImpulse,
    pub locked_axes: LockedAxes,
    pub damping: Damping,
}

impl Rapier2dCharacterBundle {
    /// Create a new character bundle with rotation enabled for upright torque.
    pub fn new() -> Self {
        Self {
            rigid_body: RigidBody::Dynamic,
            velocity: Velocity::default(),
            external_force: ExternalForce::default(),
            external_impulse: ExternalImpulse::default(),
            locked_axes: LockedAxes::empty(),
            damping: Damping {
                linear_damping: 0.5,
                angular_damping: 1.0,
            },
        }
    }

    /// Create a character bundle with rotation locked.
    ///
    /// Use this when the character's rotation is controlled by other systems
    /// (e.g., KeepUpright) or when upright torque is disabled.
    pub fn rotation_locked() -> Self {
        Self {
            locked_axes: LockedAxes::ROTATION_LOCKED,
            ..Self::new()
        }
    }

    pub fn with_body(mut self, body: RigidBody) -> Self {
        self.rigid_body = body;
        self
    }

    pub fn with_damping(mut self, linear: f32, angular: f32) -> Self {
        self.damping = Damping {
            linear_damping: linear,
            angular_damping: angular,
        };
        self
    }

    /// Builder: set locked axes.
    pub fn with_locked_axes(mut self, axes: LockedAxes) -> Self {
        self.locked_axes = axes;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_app() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
        app.insert_resource(Time::<Fixed>::from_hz(60.0));
        app
    }

    #[test]
    fn rapier_backend_get_position() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn((Transform::from_xyz(100.0, 200.0, 0.0), RigidBody::Dynamic))
            .id();

        app.update();

        let pos = Rapier2dBackend::get_position(app.world(), entity);
        assert!((pos.x - 100.0).abs() < 0.01);
        assert!((pos.y - 200.0).abs() < 0.01);
    }

    #[test]
    fn rapier_backend_velocity() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                RigidBody::Dynamic,
                Velocity::linear(Vec2::new(50.0, 30.0)),
            ))
            .id();

        app.update();

        let vel = Rapier2dBackend::get_velocity(app.world(), entity);
        assert!((vel.x - 50.0).abs() < 0.01);
        assert!((vel.y - 30.0).abs() < 0.01);

        Rapier2dBackend::set_velocity(app.world_mut(), entity, Vec2::new(100.0, 0.0));

        let vel = Rapier2dBackend::get_velocity(app.world(), entity);
        assert!((vel.x - 100.0).abs() < 0.01);
        assert!(vel.y.abs() < 0.01);
    }

    #[test]
    fn rapier_character_bundle_creates_valid_entity() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                Rapier2dCharacterBundle::new(),
                Collider::capsule_y(8.0, 4.0),
            ))
            .id();

        app.update();

        assert!(app.world().get::<RigidBody>(entity).is_some());
        assert!(app.world().get::<Velocity>(entity).is_some());
        assert!(app.world().get::<ExternalForce>(entity).is_some());
        assert!(app.world().get::<LockedAxes>(entity).is_some());
    }
}
