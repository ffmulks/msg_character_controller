//! Rapier2D physics backend implementation.
//!
//! This module provides the physics backend for Bevy Rapier2D.
//! Enable with the `rapier2d` feature.

use bevy::prelude::*;
use bevy_rapier2d::parry::shape::Segment;
use bevy_rapier2d::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, CharacterOrientation, ControllerConfig, StairConfig};
use crate::detection::SensorCast;

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

    fn get_gravity(_world: &World, _entity: Entity) -> Vec2 {
        // Gravity is stored in CharacterController.gravity
        Vec2::ZERO
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
            (
                rapier_ground_detection,
                rapier_wall_detection,
                rapier_ceiling_detection,
            )
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


/// Get the distance from collider center to bottom for a given collider.
/// For capsules, this is half_height + radius.
fn get_collider_bottom_offset(collider: &Collider) -> f32 {
    // Try to get capsule shape parameters
    if let Some(capsule) = collider.as_capsule() {
        // Capsule: half-length of segment + radius
        // For capsule_y(half_height, radius), the segment endpoints are at y = ±half_height
        let segment = capsule.segment();
        let half_height = (segment.a().y - segment.b().y).abs() / 2.0;
        half_height + capsule.radius()
    } else if let Some(ball) = collider.as_ball() {
        // Ball: just the radius
        ball.radius()
    } else if let Some(cuboid) = collider.as_cuboid() {
        // Cuboid: half the height (y dimension)
        cuboid.half_extents().y
    } else {
        // Unknown shape: use 0 as fallback (float_height measured from center)
        0.0
    }
}

/// Rapier-specific ground detection system using shapecast.
/// Inherits collision layers from the parent entity's collider.
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
            &mut CharacterController,
            Option<&CollisionGroups>,
            Option<&Collider>,
        ),
    >,
    time: Res<Time<Fixed>>,
) {
    let Ok(context) = rapier_context.single() else {
        return;
    };

    let default_orientation = CharacterOrientation::default();
    let dt = time.delta_secs();

    for (entity, transform, config, orientation_opt, stair_config, velocity, mut controller, collision_groups, collider) in
        &mut q_controllers
    {
        let position = transform.translation().xy();

        // Update collider_bottom_offset from actual collider dimensions
        controller.collider_bottom_offset = collider
            .map(get_collider_bottom_offset)
            .unwrap_or(0.0);

        // Get orientation (use default if component not present)
        let orientation = orientation_opt.unwrap_or(&default_orientation);
        let down = orientation.down();
        let right = orientation.right();

        // Inherit collision groups from parent's collider
        let (memberships, filters) = collision_groups
            .map(|cg| (cg.memberships, cg.filters))
            .unwrap_or((Group::ALL, Group::ALL));

        // Build query filter that inherits collision layers
        let filter = QueryFilter::default()
            .exclude_rigid_body(entity)
            .exclude_sensors()
            .groups(CollisionGroups::new(memberships, filters));

        // Create a flat horizontal segment for ground detection
        let half_width = config.ground_cast_width / 2.0;
        let segment_a = right * -half_width;
        let segment_b = right * half_width;
        let shape = Segment::new(segment_a.into(), segment_b.into());

        // Compute rotation angle for the shape to align with character orientation
        let shape_rotation = orientation.angle() - std::f32::consts::FRAC_PI_2;

        // Calculate effective float height (from center) and ground cast length
        let effective_height = controller.effective_float_height(config);
        let ground_cast_length = effective_height * config.ground_cast_multiplier;

        // Perform shapecast - use derived ground cast length
        let shape_result = context.cast_shape(
            position,
            shape_rotation,
            down,
            &shape,
            ShapeCastOptions {
                max_time_of_impact: ground_cast_length,
                stop_at_penetration: false,
                ..default()
            },
            filter,
        );

        // Store previous time_since_grounded
        let prev_time_since_grounded = controller.time_since_grounded;

        // Reset ground detection state
        controller.reset_detection_state();

        // Update from shapecast result
        if let Some((hit_entity, hit)) = shape_result {
            let normal = hit.details.map(|d| d.normal1).unwrap_or(orientation.up());
            let up = orientation.up();
            let dot = normal.dot(up).clamp(-1.0, 1.0);
            let slope_angle = dot.acos();

            controller.ground_detected = true;
            controller.ground_distance = hit.time_of_impact;
            controller.ground_normal = normal;
            controller.ground_contact_point = position + down * hit.time_of_impact;
            controller.slope_angle = slope_angle;
            controller.ground_entity = Some(hit_entity);

            // Check for stairs if enabled
            let is_walkable = slope_angle <= config.max_slope_angle;
            if let Some(stair) = stair_config {
                if stair.enabled && !is_walkable {
                    if let Some(step_height) = check_stair_step(
                        &context,
                        entity,
                        position,
                        velocity.linvel,
                        orientation,
                        stair,
                        (memberships, filters),
                    ) {
                        controller.step_detected = true;
                        controller.step_height = step_height;
                    }
                }
            }
        }

        // Update grounded state using effective float height (accounts for collider dimensions)
        controller.is_grounded = controller.ground_detected
            && controller.ground_distance <= effective_height + config.cling_distance;

        // Update time since grounded
        if controller.is_grounded {
            controller.time_since_grounded = 0.0;
        } else {
            controller.time_since_grounded = prev_time_since_grounded + dt;
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
    collision_groups: (Group, Group),
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
        .exclude_sensors()
        .groups(CollisionGroups::new(collision_groups.0, collision_groups.1));

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
/// Inherits collision layers from the parent entity's collider.
fn rapier_wall_detection(
    rapier_context: ReadRapierContext,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &ControllerConfig,
        Option<&CharacterOrientation>,
        &mut CharacterController,
        Option<&CollisionGroups>,
    )>,
) {
    let Ok(context) = rapier_context.single() else {
        return;
    };

    let default_orientation = CharacterOrientation::default();

    for (entity, transform, config, orientation_opt, mut controller, collision_groups) in
        &mut q_controllers
    {
        let position = transform.translation().xy();

        // Get orientation (use default if component not present)
        let orientation = orientation_opt.unwrap_or(&default_orientation);
        let left = orientation.left();
        let right = orientation.right();
        let up = orientation.up();

        // Inherit collision groups from parent's collider
        let (memberships, filters) = collision_groups
            .map(|cg| (cg.memberships, cg.filters))
            .unwrap_or((Group::ALL, Group::ALL));

        let filter = QueryFilter::default()
            .exclude_rigid_body(entity)
            .exclude_sensors()
            .groups(CollisionGroups::new(memberships, filters));

        // Create a vertical segment for wall detection (aligned with up direction)
        let half_height = config.wall_cast_height / 2.0;
        let segment_a = up * -half_height;
        let segment_b = up * half_height;
        let shape = Segment::new(segment_a.into(), segment_b.into());

        // Compute rotation angle for the shape
        let shape_rotation = orientation.angle();

        // Use derived wall cast length
        let shape_opts = ShapeCastOptions {
            max_time_of_impact: config.wall_cast_length(),
            stop_at_penetration: false,
            ..default()
        };

        // Shapecast left - touching when within wall cast length
        if let Some((_, hit)) =
            context.cast_shape(position, shape_rotation, left, &shape, shape_opts, filter)
        {
            controller.touching_left_wall = true;
            controller.left_wall_normal = hit.details.map(|d| d.normal1).unwrap_or(right);
        }

        // Shapecast right - touching when within wall cast length
        if let Some((_, hit)) =
            context.cast_shape(position, shape_rotation, right, &shape, shape_opts, filter)
        {
            controller.touching_right_wall = true;
            controller.right_wall_normal = hit.details.map(|d| d.normal1).unwrap_or(left);
        }
    }
}

/// Rapier-specific ceiling detection system using shapecast.
/// Inherits collision layers from the parent entity's collider.
fn rapier_ceiling_detection(
    rapier_context: ReadRapierContext,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &ControllerConfig,
        Option<&CharacterOrientation>,
        &mut CharacterController,
        Option<&CollisionGroups>,
    )>,
) {
    let Ok(context) = rapier_context.single() else {
        return;
    };

    let default_orientation = CharacterOrientation::default();

    for (entity, transform, config, orientation_opt, mut controller, collision_groups) in
        &mut q_controllers
    {
        let position = transform.translation().xy();

        // Get orientation
        let orientation = orientation_opt.unwrap_or(&default_orientation);
        let up = orientation.up();
        let right = orientation.right();

        // Inherit collision groups from parent's collider
        let (memberships, filters) = collision_groups
            .map(|cg| (cg.memberships, cg.filters))
            .unwrap_or((Group::ALL, Group::ALL));

        let filter = QueryFilter::default()
            .exclude_rigid_body(entity)
            .exclude_sensors()
            .groups(CollisionGroups::new(memberships, filters));

        // Create a horizontal segment for ceiling detection
        let half_width = config.ceiling_cast_width / 2.0;
        let segment_a = right * -half_width;
        let segment_b = right * half_width;
        let shape = Segment::new(segment_a.into(), segment_b.into());

        let shape_rotation = orientation.angle() - std::f32::consts::FRAC_PI_2;

        // Calculate ceiling cast length using effective float height
        let effective_height = controller.effective_float_height(config);
        let ceiling_cast_length = effective_height * config.ceiling_cast_multiplier;

        // Shapecast upward - use derived ceiling cast length
        if let Some((_, hit)) = context.cast_shape(
            position,
            shape_rotation,
            up,
            &shape,
            ShapeCastOptions {
                max_time_of_impact: ceiling_cast_length,
                stop_at_penetration: false,
                ..default()
            },
            filter,
        ) {
            controller.touching_ceiling = true;
            controller.ceiling_normal = hit.details.map(|d| d.normal1).unwrap_or(orientation.down());
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
