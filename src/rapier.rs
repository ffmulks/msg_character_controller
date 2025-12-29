//! Rapier2D physics backend implementation.
//!
//! This module provides the physics backend for Bevy Rapier2D.
//! Enable with the `rapier2d` feature.

use bevy::prelude::*;
use bevy_rapier2d::geometry::Group;
use bevy_rapier2d::parry::shape::Segment;
use bevy_rapier2d::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::collision::CollisionData;
use crate::config::CharacterController;

/// Rapier2D physics backend for the character controller.
///
/// This backend uses `bevy_rapier2d` for physics operations including
/// force application and velocity manipulation. Collision detection
/// (shapecasting/raycasting) is handled by dedicated Rapier systems
/// that receive `RapierContext` as a system parameter.
pub struct Rapier2dBackend;

impl CharacterPhysicsBackend for Rapier2dBackend {
    type VelocityComponent = Velocity;

    fn plugin() -> impl Plugin {
        Rapier2dBackendPlugin
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
        // Accumulate into CharacterController instead of directly modifying ExternalForce.
        // Forces will be applied to ExternalForce at the end of the frame by finalize_controller_forces.
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.add_force(force);
        }
    }

    fn apply_torque(world: &mut World, entity: Entity, torque: f32) {
        // Accumulate into CharacterController instead of directly modifying ExternalForce.
        // Torque will be applied to ExternalForce at the end of the frame by finalize_controller_forces.
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.add_torque(torque);
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

    fn get_collision_groups(world: &World, entity: Entity) -> Option<(u32, u32)> {
        world
            .get::<CollisionGroups>(entity)
            .map(|cg| (cg.memberships.bits(), cg.filters.bits()))
    }

    fn get_collider_bottom_offset(world: &World, entity: Entity) -> f32 {
        world
            .get::<Collider>(entity)
            .map(get_collider_bottom_offset)
            .unwrap_or(0.0)
    }

    fn get_mass(world: &World, entity: Entity) -> f32 {
        let props = world
            .get::<ReadMassProperties>(entity)
            .expect("Entity must have ReadMassProperties component for mass calculation");
        let mass = props.mass;
        assert!(
            mass > 0.0 && mass.is_finite(),
            "Entity mass must be positive and finite, got: {}",
            mass
        );
        mass
    }

    fn get_principal_inertia(world: &World, entity: Entity) -> f32 {
        let props = world
            .get::<ReadMassProperties>(entity)
            .expect("Entity must have ReadMassProperties component for inertia calculation");
        let inertia = props.principal_inertia;
        assert!(
            inertia > 0.0 && inertia.is_finite(),
            "Entity principal inertia must be positive and finite, got: {}",
            inertia
        );
        inertia
    }
}

/// Plugin that sets up Rapier2D-specific systems for the character controller.
pub struct Rapier2dBackendPlugin;

impl Plugin for Rapier2dBackendPlugin {
    fn build(&self, app: &mut App) {
        use crate::CharacterControllerSet;

        // Phase 1: Preparation - Clear forces from previous frame
        app.add_systems(
            FixedUpdate,
            clear_controller_forces.in_set(CharacterControllerSet::Preparation),
        );

        // Phase 3: Sensors - Rapier-specific detection systems
        // Ground detection must run first because it sets collider_bottom_offset
        // which ceiling detection uses for capsule_half_height().
        // Wall and ceiling detection can run in parallel after ground detection.
        app.add_systems(
            FixedUpdate,
            (
                rapier_ground_detection,
                (rapier_wall_detection, rapier_ceiling_detection),
            )
                .chain()
                .in_set(CharacterControllerSet::Sensors),
        );

        // Phase 6: Final Application - Apply accumulated forces to physics
        app.add_systems(
            FixedUpdate,
            apply_controller_forces.in_set(CharacterControllerSet::FinalApplication),
        );
    }
}

/// Get the distance from collider center to bottom for a given collider.
/// For capsules, this is half_height + radius.
pub fn get_collider_bottom_offset(collider: &Collider) -> f32 {
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

// Rapier-specific detection systems that use RapierContext as a system parameter
use crate::config::{CharacterOrientation, ControllerConfig, StairConfig};

/// Perform a shapecast using RapierContext.
fn rapier_shapecast(
    context: &RapierContext,
    origin: Vec2,
    direction: Vec2,
    max_distance: f32,
    shape_width: f32,
    shape_height: f32,
    shape_rotation: f32,
    exclude_entity: Entity,
    collision_groups: Option<(Group, Group)>,
) -> Option<CollisionData> {
    // Determine if we're creating a horizontal or vertical segment based on dimensions
    let shape = if shape_height > shape_width {
        // Vertical segment (for wall detection)
        let half_height = shape_height / 2.0;
        let segment_a = Vec2::new(0.0, -half_height);
        let segment_b = Vec2::new(0.0, half_height);
        Segment::new(segment_a.into(), segment_b.into())
    } else {
        // Horizontal segment (for ground/ceiling detection)
        let half_width = shape_width / 2.0;
        let segment_a = Vec2::new(-half_width, 0.0);
        let segment_b = Vec2::new(half_width, 0.0);
        Segment::new(segment_a.into(), segment_b.into())
    };

    // Create filter to exclude the casting entity
    let mut filter = QueryFilter::default()
        .exclude_rigid_body(exclude_entity)
        .exclude_sensors();

    // Apply collision groups if provided
    if let Some((memberships, filters)) = collision_groups {
        filter = filter.groups(CollisionGroups::new(memberships, filters));
    }

    // Perform the shapecast
    context
        .cast_shape(
            origin,
            shape_rotation,
            direction,
            &shape,
            ShapeCastOptions {
                max_time_of_impact: max_distance,
                stop_at_penetration: false,
                ..default()
            },
            filter,
        )
        .map(|(hit_entity, hit)| {
            // Extract normal from hit details or use default
            let normal = hit.details.map(|d| d.normal1).unwrap_or(-direction);
            // Calculate hit point
            let hit_point = origin + direction * hit.time_of_impact;
            CollisionData::new(hit.time_of_impact, normal, hit_point, Some(hit_entity))
        })
}

/// Perform a raycast using RapierContext.
fn rapier_raycast(
    context: &RapierContext,
    origin: Vec2,
    direction: Vec2,
    max_distance: f32,
    exclude_entity: Entity,
    collision_groups: Option<(Group, Group)>,
) -> Option<CollisionData> {
    // Create filter to exclude the casting entity
    let mut filter = QueryFilter::default()
        .exclude_rigid_body(exclude_entity)
        .exclude_sensors();

    // Apply collision groups if provided
    if let Some((memberships, filters)) = collision_groups {
        filter = filter.groups(CollisionGroups::new(memberships, filters));
    }

    // Perform the raycast
    context
        .cast_ray(
            origin,
            direction,
            max_distance,
            true, // solid = true for solid hits
            filter,
        )
        .map(|(hit_entity, toi)| {
            // Calculate hit point
            let hit_point = origin + direction * toi;
            // For a simple ray, we approximate the normal as opposite of ray direction
            let normal = -direction;
            CollisionData::new(toi, normal, hit_point, Some(hit_entity))
        })
}

use crate::intent::MovementIntent;

/// Rapier-specific ground detection system using shapecast.
///
/// Floor raycast covers: riding_height + ground_tolerance
/// (which is float_height + capsule_half_height + ground_tolerance)
fn rapier_ground_detection(
    rapier_context: ReadRapierContext,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &ControllerConfig,
        Option<&CharacterOrientation>,
        Option<&MovementIntent>,
        &mut CharacterController,
        Option<&CollisionGroups>,
        Option<&Collider>,
    )>,
    time: Res<Time<Fixed>>,
) {
    let Ok(context) = rapier_context.single() else {
        return;
    };

    let default_orientation = CharacterOrientation::default();
    let dt = time.delta_secs();

    for (
        entity,
        transform,
        config,
        orientation_opt,
        movement_intent,
        mut controller,
        collision_groups,
        collider,
    ) in &mut q_controllers
    {
        let position = transform.translation().xy();

        // Get collider radius for stair detection offset
        let radius = collider.map(get_collider_radius).unwrap_or(0.0);

        // Update collider_bottom_offset from actual collider dimensions
        controller.collider_bottom_offset = collider.map(get_collider_bottom_offset).unwrap_or(0.0);

        // Get orientation (use default if component not present)
        let orientation = orientation_opt.unwrap_or(&default_orientation);
        let down = orientation.down();

        // Inherit collision groups from parent's collider
        let collision_groups_tuple = collision_groups.map(|cg| (cg.memberships, cg.filters));

        // Calculate ground cast length:
        // riding_height + ground_tolerance = float_height + capsule_half_height + ground_tolerance
        // Add a small buffer (1.0) to avoid edge cases with floating point precision
        let riding_height = controller.riding_height(config);
        let ground_cast_length = riding_height + config.ground_tolerance + 1.0;

        // Compute rotation angle for the shape to align with character orientation
        let shape_rotation = orientation.angle() - std::f32::consts::FRAC_PI_2;

        // Store previous time_since_grounded
        let prev_time_since_grounded = controller.time_since_grounded;

        // Reset detection state
        controller.reset_detection_state();

        // Perform shapecast for ground detection
        if let Some(ground_hit) = rapier_shapecast(
            &context,
            position,
            down,
            ground_cast_length,
            config.ground_cast_width,
            0.0, // height not used for ground detection
            shape_rotation,
            entity,
            collision_groups_tuple,
        ) {
            let normal = ground_hit.normal;
            let up = orientation.up();
            let dot = normal.dot(up).clamp(-1.0, 1.0);
            let slope_angle = dot.acos();

            // Store floor collision data
            controller.floor = Some(ground_hit);
            controller.slope_angle = slope_angle;
        }

        // Check for stairs if enabled and we have movement intent
        if let (Some(stair), Some(intent)) = (controller.stair_config.as_ref(), movement_intent) {
            if stair.enabled && intent.is_walking() {
                if let Some(step_height) = check_stair_step(
                    &context,
                    entity,
                    position,
                    radius,
                    config.float_height,
                    intent,
                    orientation,
                    stair,
                    collision_groups_tuple,
                ) {
                    controller.step_detected = true;
                    controller.step_height = step_height;
                }
            }
        }

        // Update time since grounded
        let is_grounded = controller.is_grounded(config);
        if is_grounded {
            controller.time_since_grounded = 0.0;
        } else {
            controller.time_since_grounded = prev_time_since_grounded + dt;
        }
    }
}

/// Check for a climbable stair in front of the character using movement intent.
///
/// This function casts a shapecast downward from a position in front of the character
/// (in the direction of movement intent) to detect steps.
///
/// Returns Some(step_height) if a climbable stair is detected, where step_height is
/// the height above the current ground level that needs to be climbed.
fn check_stair_step(
    context: &RapierContext,
    entity: Entity,
    position: Vec2,
    collider_radius: f32,
    float_height: f32,
    intent: &MovementIntent,
    orientation: &CharacterOrientation,
    config: &StairConfig,
    collision_groups: Option<(Group, Group)>,
) -> Option<f32> {
    let down = orientation.down();
    let up = orientation.up();
    let right = orientation.right();

    // Determine movement direction from intent
    let walk_direction = intent.walk;
    if walk_direction.abs() < 0.001 {
        return None; // No horizontal intent
    }

    let move_dir = right * walk_direction.signum();

    // Calculate the cast origin: position + move_dir * (radius + stair_cast_offset)
    // This places the cast just outside the collider in the movement direction
    let cast_offset = collider_radius + config.stair_cast_offset;
    let cast_origin = position + move_dir * cast_offset;

    // Cast downward from this position to detect the ground in front
    // Cast distance should be enough to detect steps up to max_climb_height below our current position
    // plus some buffer for floating height variations
    let cast_distance = config.max_climb_height + float_height + config.stair_tolerance + 2.0;

    // Use shapecast for more reliable detection
    let shape_rotation = orientation.angle() - std::f32::consts::FRAC_PI_2;

    let stair_hit = rapier_shapecast(
        context,
        cast_origin,
        down,
        cast_distance,
        config.stair_cast_width,
        0.0, // height not used for downward detection
        shape_rotation,
        entity,
        collision_groups,
    )?;

    // Calculate the ground height at the stair position relative to our current height
    // stair_hit.distance is how far down from cast_origin we hit
    // The step surface is at: cast_origin + down * stair_hit.distance
    let step_surface_point = cast_origin + down * stair_hit.distance;

    // Get the current ground level under the character (from center position)
    // We need to cast down from our current position to get the floor distance
    let current_ground_hit = rapier_shapecast(
        context,
        position,
        down,
        cast_distance,
        config.stair_cast_width,
        0.0, // height not used for ground detection
        shape_rotation,
        entity,
        collision_groups,
    );

    let current_ground_distance = current_ground_hit
        .map(|h| h.distance)
        .unwrap_or(f32::MAX);

    // Calculate step height: how much higher is the step surface compared to our current ground?
    // The step is at cast_origin + down * stair_hit.distance
    // Our ground is at position + down * current_ground_distance
    // The height difference in the "up" direction:
    let step_height_in_up = (step_surface_point - (position + down * current_ground_distance)).dot(up);

    // step_height_in_up will be positive if the step is above our current ground
    // (since up points up, and step surface is higher than current ground)
    let step_height = -step_height_in_up; // Negate because if step is above, dot product is negative

    // Check if the step height is within climbable range:
    // - Higher than float_height + tolerance (needs climbing, not just spring)
    // - Lower than max_climb_height (not too high to climb)
    if step_height > float_height + config.stair_tolerance && step_height <= config.max_climb_height {
        // Also verify the step has adequate depth (horizontal surface)
        // Check that the normal is mostly pointing up (it's a floor, not a wall)
        let normal_up_component = stair_hit.normal.dot(up);
        if normal_up_component > 0.7 {
            // At least 45 degrees from horizontal
            return Some(step_height);
        }
    }

    None
}

/// Get the capsule radius from a collider.
fn get_collider_radius(collider: &Collider) -> f32 {
    if let Some(capsule) = collider.as_capsule() {
        capsule.radius()
    } else if let Some(ball) = collider.as_ball() {
        ball.radius()
    } else if let Some(cuboid) = collider.as_cuboid() {
        // Use half the width as an approximation
        cuboid.half_extents().x
    } else {
        0.0
    }
}

/// Rapier-specific wall detection system using shapecast.
///
/// Wall cast length: cling_distance + radius
fn rapier_wall_detection(
    rapier_context: ReadRapierContext,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &ControllerConfig,
        Option<&CharacterOrientation>,
        &mut CharacterController,
        Option<&CollisionGroups>,
        Option<&Collider>,
    )>,
) {
    let Ok(context) = rapier_context.single() else {
        return;
    };

    let default_orientation = CharacterOrientation::default();

    for (entity, transform, config, orientation_opt, mut controller, collision_groups, collider) in
        &mut q_controllers
    {
        let position = transform.translation().xy();

        // Get orientation (use default if component not present)
        let orientation = orientation_opt.unwrap_or(&default_orientation);
        let left = orientation.left();
        let right = orientation.right();

        // Inherit collision groups from parent's collider
        let collision_groups_tuple = collision_groups.map(|cg| (cg.memberships, cg.filters));

        // Compute rotation angle for the shape
        let shape_rotation = orientation.angle();

        // Wall cast length: cling_distance + radius + small buffer for precision
        let radius = collider.map(get_collider_radius).unwrap_or(0.0);
        let wall_cast_length = config.cling_distance + radius + 1.0;

        // Shapecast left
        if let Some(left_hit) = rapier_shapecast(
            &context,
            position,
            left,
            wall_cast_length,
            0.0, // width not used for wall detection
            config.wall_cast_height,
            shape_rotation,
            entity,
            collision_groups_tuple,
        ) {
            controller.left_wall = Some(left_hit);
        }

        // Shapecast right
        if let Some(right_hit) = rapier_shapecast(
            &context,
            position,
            right,
            wall_cast_length,
            0.0, // width not used for wall detection
            config.wall_cast_height,
            shape_rotation,
            entity,
            collision_groups_tuple,
        ) {
            controller.right_wall = Some(right_hit);
        }
    }
}

/// Rapier-specific ceiling detection system using shapecast.
///
/// Ceiling cast length: cling_distance + capsule_half_height
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

        // Inherit collision groups from parent's collider
        let collision_groups_tuple = collision_groups.map(|cg| (cg.memberships, cg.filters));

        let shape_rotation = orientation.angle() - std::f32::consts::FRAC_PI_2;

        // Ceiling cast length: cling_distance + capsule_half_height + small buffer for precision
        let ceiling_cast_length = config.cling_distance + controller.capsule_half_height() + 1.0;

        // Shapecast upward
        if let Some(ceiling_hit) = rapier_shapecast(
            &context,
            position,
            up,
            ceiling_cast_length,
            config.ceiling_cast_width,
            0.0, // height not used for ceiling detection
            shape_rotation,
            entity,
            collision_groups_tuple,
        ) {
            controller.ceiling = Some(ceiling_hit);
        }
    }
}

/// Clear controller forces at the start of each frame.
///
/// This system runs BEFORE any controller force systems. It:
/// 1. Subtracts the forces we applied last frame from ExternalForce
/// 2. Clears the accumulators for the new frame
///
/// This ensures that external user forces are preserved while our forces
/// are "isolated" between frames.
pub fn clear_controller_forces(mut q: Query<(&mut ExternalForce, &mut CharacterController)>) {
    for (mut ext_force, mut controller) in &mut q {
        // Get the forces we applied last frame and clear for the new frame
        let (force_to_subtract, torque_to_subtract) = controller.prepare_new_frame();

        // Subtract our previously applied forces from ExternalForce
        // This restores ExternalForce to the "external-only" state
        ext_force.force -= force_to_subtract;
        ext_force.torque -= torque_to_subtract;
    }
}

/// Apply controller forces at the end of each frame.
///
/// This system runs AFTER all controller force systems. It:
/// 1. Applies accumulated forces to ExternalForce
/// 2. Stores what we applied for next frame's subtraction
///
/// This ensures our forces are integrated by Rapier's physics step.
pub fn apply_controller_forces(mut q: Query<(&mut ExternalForce, &mut CharacterController)>) {
    for (mut ext_force, mut controller) in &mut q {
        // Get accumulated forces and prepare for next frame
        let (force_to_apply, torque_to_apply) = controller.finalize_frame();

        // Apply our accumulated forces to ExternalForce
        ext_force.force += force_to_apply;
        ext_force.torque += torque_to_apply;
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
    /// Computed mass properties - Rapier will update this based on collider
    pub mass_properties: ReadMassProperties,
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
            // Rapier will update this based on collider after first physics step
            mass_properties: ReadMassProperties::default(),
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
