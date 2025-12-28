//! Detection result structures.
//!
//! These structures hold the results of physics queries (raycasts) used
//! for ground detection, wall detection, and stair stepping.

use bevy::prelude::*;

/// Information about a raycast/shapecast collision.
#[derive(Debug, Clone, Copy, Default)]
pub struct CollisionData {
    /// Distance to the hit point (if hit).
    pub distance: f32,
    /// Normal of the surface at hit point.
    pub normal: Vec2,
    /// World position of the hit point.
    pub point: Vec2,
    /// Entity that was hit (if any).
    pub entity: Option<Entity>,
}

impl CollisionData {
    /// Create a collisionresult.
    pub fn new(distance: f32, normal: Vec2, point: Vec2, entity: Option<Entity>) -> Self {
        Self {
            distance,
            normal,
            point,
            entity,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_cast_hit() {
        let cast = CollisionData::new(5.0, Vec2::Y, Vec2::new(10.0, 0.0), None);

        assert_eq!(cast.distance, 5.0);
        assert_eq!(cast.normal, Vec2::Y);
        assert_eq!(cast.point, Vec2::new(10.0, 0.0));
    }

    #[test]
    fn sensor_cast_with_entity() {
        let entity = Entity::from_raw(42);
        let cast = CollisionData::new(3.0, Vec2::X, Vec2::ZERO, Some(entity));

        assert_eq!(cast.entity, Some(entity));
    }
}
