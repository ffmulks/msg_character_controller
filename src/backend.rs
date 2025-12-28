//! Physics backend abstraction.
//!
//! This module defines the trait that physics backends must implement
//! to work with the character controller. This allows easy swapping
//! between physics engines (Rapier2D, XPBD, custom, etc.).

use bevy::prelude::*;

use crate::collision::CollisionData;

/// Trait for physics backend implementations.
///
/// Implement this trait to integrate a physics engine with the character
/// controller. The backend handles all physics operations like raycasting,
/// force application, and velocity manipulation.
///
/// # Example
///
/// For an example implementation, see the `rapier` module's `Rapier2dBackend`
/// which implements this trait for Bevy Rapier2D.
///
/// ```rust
/// use bevy::prelude::*;
/// use msg_character_controller::prelude::*;
///
/// // The trait is implemented by physics backends like Rapier2dBackend
/// #[cfg(feature = "rapier2d")]
/// fn example_usage() {
///     // Access backend methods statically
///     // In practice, these are called by the controller systems
/// }
/// ```
pub trait CharacterPhysicsBackend: 'static + Send + Sync {
    /// The velocity component type used by this backend.
    type VelocityComponent: Component;

    /// Returns the plugin that sets up this backend.
    fn plugin() -> impl Plugin;

    /// Perform a shapecast (sweep a shape along a direction) and return the result.
    ///
    /// This casts a line/capsule shape along a direction, providing better
    /// ground detection than a single raycast. The shape's normal at the hit
    /// point is used for slope detection.
    ///
    /// # Arguments
    /// * `world` - The ECS world for queries
    /// * `origin` - Shape origin in world space
    /// * `direction` - Cast direction (should be normalized)
    /// * `max_distance` - Maximum cast distance
    /// * `shape_width` - Width of the cast shape (half-width of the line)
    /// * `shape_height` - Height of the cast shape (for wall detection)
    /// * `shape_rotation` - Rotation of the shape in radians
    /// * `exclude_entity` - Entity to exclude from cast (usually self)
    /// * `collision_groups` - Optional collision groups for filtering (memberships, filters)
    fn shapecast(
        world: &World,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
        shape_width: f32,
        shape_height: f32,
        shape_rotation: f32,
        exclude_entity: Entity,
        collision_groups: Option<(u32, u32)>,
    ) -> Option<CollisionData>;

    /// Perform a simple raycast (for stair detection, etc).
    ///
    /// # Arguments
    /// * `world` - The ECS world for queries
    /// * `origin` - Shape origin in world space
    /// * `direction` - Cast direction (should be normalized)
    /// * `max_distance` - Maximum cast distance
    /// * `exclude_entity` - Entity to exclude from cast (usually self)
    /// * `collision_groups` - Optional collision groups for filtering (memberships, filters)
    fn raycast(
        world: &World,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
        exclude_entity: Entity,
        collision_groups: Option<(u32, u32)>,
    ) -> Option<CollisionData>;

    /// Get the current velocity of an entity.
    fn get_velocity(world: &World, entity: Entity) -> Vec2;

    /// Set the velocity of an entity.
    fn set_velocity(world: &mut World, entity: Entity, velocity: Vec2);

    /// Apply an impulse to an entity.
    ///
    /// Impulse is an instantaneous change in momentum (velocity).
    fn apply_impulse(world: &mut World, entity: Entity, impulse: Vec2);

    /// Apply a force to an entity.
    ///
    /// Force is applied over the physics timestep.
    fn apply_force(world: &mut World, entity: Entity, force: Vec2);

    /// Apply a torque to an entity.
    ///
    /// Torque is applied over the physics timestep. Positive values rotate
    /// counter-clockwise, negative values rotate clockwise.
    fn apply_torque(world: &mut World, entity: Entity, torque: f32);

    /// Get the current angular velocity of an entity.
    fn get_angular_velocity(world: &World, entity: Entity) -> f32;

    /// Get the current rotation angle of an entity (in radians).
    fn get_rotation(world: &World, entity: Entity) -> f32;

    /// Get the gravity vector for an entity.
    ///
    /// Returns the gravity that affects this entity. For spherical worlds,
    /// this may vary based on position.
    fn get_gravity(world: &World, entity: Entity) -> Vec2;

    /// Get the current position of an entity.
    fn get_position(world: &World, entity: Entity) -> Vec2;

    /// Get the fixed timestep delta time.
    fn get_fixed_timestep(world: &World) -> f32;

    /// Get the collision groups for an entity (memberships, filters).
    /// Returns None if the entity doesn't have collision groups.
    fn get_collision_groups(_world: &World, _entity: Entity) -> Option<(u32, u32)> {
        // Default implementation returns None
        None
    }

    /// Get the collider bottom offset for an entity.
    /// This is the distance from the collider center to its bottom.
    fn get_collider_bottom_offset(_world: &World, _entity: Entity) -> f32 {
        // Default implementation returns 0
        0.0
    }

    /// Get the mass of an entity.
    ///
    /// Used to scale forces so that config parameters produce consistent
    /// acceleration regardless of actual body mass.
    fn get_mass(_world: &World, _entity: Entity) -> f32 {
        // Default implementation returns 1.0 (no scaling)
        1.0
    }

    /// Get the principal moment of inertia of an entity.
    ///
    /// In 2D this is a scalar. Used to scale torques so that config parameters
    /// produce consistent angular acceleration regardless of actual body inertia.
    fn get_principal_inertia(_world: &World, _entity: Entity) -> f32 {
        // Default implementation returns 1.0 (no scaling)
        1.0
    }
}

/// Empty plugin for backends that don't need additional setup.
pub struct NoOpBackendPlugin;

impl Plugin for NoOpBackendPlugin {
    fn build(&self, _app: &mut App) {}
}

/// Helper struct for building raycasts.
#[derive(Debug, Clone, Copy)]
pub struct RaycastRequest {
    /// Origin point of the ray.
    pub origin: Vec2,
    /// Direction of the ray (should be normalized).
    pub direction: Vec2,
    /// Maximum distance to cast.
    pub max_distance: f32,
    /// Entity to exclude from results.
    pub exclude: Option<Entity>,
}

impl RaycastRequest {
    /// Create a new raycast request.
    pub fn new(origin: Vec2, direction: Vec2, max_distance: f32) -> Self {
        Self {
            origin,
            direction: direction.normalize_or_zero(),
            max_distance,
            exclude: None,
        }
    }

    /// Exclude an entity from the raycast.
    pub fn excluding(mut self, entity: Entity) -> Self {
        self.exclude = Some(entity);
        self
    }
}
