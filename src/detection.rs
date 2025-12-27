//! Detection result structures.
//!
//! These structures hold the results of physics queries (raycasts) used
//! for ground detection, wall detection, and stair stepping.

use bevy::prelude::*;

/// Information about a raycast result.
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorCast {
    /// Whether the raycast hit something.
    pub hit: bool,
    /// Distance to the hit point (if hit).
    pub distance: f32,
    /// Normal of the surface at hit point.
    pub normal: Vec2,
    /// World position of the hit point.
    pub point: Vec2,
    /// Entity that was hit (if any).
    pub entity: Option<Entity>,
}

impl SensorCast {
    /// Create an empty (no hit) result.
    pub fn miss() -> Self {
        Self::default()
    }

    /// Create a hit result.
    pub fn hit(distance: f32, normal: Vec2, point: Vec2, entity: Option<Entity>) -> Self {
        Self {
            hit: true,
            distance,
            normal,
            point,
            entity,
        }
    }
}

/// Ground detection information.
///
/// Computed by the ground detection system and stored as a component.
/// Contains all information needed for floating spring and slope handling.
#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct GroundInfo {
    /// Whether ground was detected within range.
    pub detected: bool,

    /// Distance to ground surface.
    pub distance: f32,

    /// Ground surface normal (points away from surface).
    pub normal: Vec2,

    /// Ground contact point in world space.
    pub contact_point: Vec2,

    /// Slope angle in radians (0 = flat, positive = uphill in movement direction).
    pub slope_angle: f32,

    /// Whether the slope is walkable (within max_slope_angle).
    pub is_walkable: bool,

    /// Whether a valid step was detected (for stair stepping).
    pub step_detected: bool,

    /// Height of the detected step (if any).
    pub step_height: f32,

    /// Time since last grounded (for coyote time).
    pub time_since_grounded: f32,

    /// Entity that was hit (ground entity).
    pub ground_entity: Option<Entity>,
}

impl GroundInfo {
    /// Check if the character is considered grounded.
    ///
    /// Returns true if ground is detected within cling distance.
    pub fn is_grounded(&self, float_height: f32, cling_distance: f32) -> bool {
        self.detected && self.distance <= float_height + cling_distance
    }

    /// Check if on a slope that requires extra handling.
    pub fn is_on_slope(&self) -> bool {
        self.detected && self.slope_angle.abs() > 0.1
    }

    /// Get the ground tangent vector (for movement direction).
    pub fn tangent(&self) -> Vec2 {
        Vec2::new(self.normal.y, -self.normal.x)
    }

    /// Get the "up" direction based on ground normal.
    pub fn up(&self) -> Vec2 {
        self.normal
    }

    /// Calculate the height error for the floating spring.
    pub fn height_error(&self, target_height: f32) -> f32 {
        if self.detected {
            target_height - self.distance
        } else {
            0.0
        }
    }
}

/// Wall detection information.
///
/// Computed by the wall detection system.
#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct WallInfo {
    /// Whether a wall is detected on the left.
    pub left_detected: bool,
    /// Distance to left wall.
    pub left_distance: f32,
    /// Left wall normal.
    pub left_normal: Vec2,

    /// Whether a wall is detected on the right.
    pub right_detected: bool,
    /// Distance to right wall.
    pub right_distance: f32,
    /// Right wall normal.
    pub right_normal: Vec2,
}

impl WallInfo {
    /// Check if any wall is detected.
    pub fn any_wall(&self) -> bool {
        self.left_detected || self.right_detected
    }

    /// Check if movement in a direction is blocked by a wall.
    pub fn is_blocked(&self, direction: f32, min_distance: f32) -> bool {
        if direction < 0.0 && self.left_detected && self.left_distance < min_distance {
            return true;
        }
        if direction > 0.0 && self.right_detected && self.right_distance < min_distance {
            return true;
        }
        false
    }

    /// Get the wall normal if touching a wall in the given direction.
    pub fn wall_normal(&self, direction: f32) -> Option<Vec2> {
        if direction < 0.0 && self.left_detected {
            Some(self.left_normal)
        } else if direction > 0.0 && self.right_detected {
            Some(self.right_normal)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== SensorCast Tests ====================

    #[test]
    fn sensor_cast_miss() {
        let cast = SensorCast::miss();
        assert!(!cast.hit);
        assert_eq!(cast.distance, 0.0);
        assert!(cast.entity.is_none());
    }

    #[test]
    fn sensor_cast_hit() {
        let cast = SensorCast::hit(5.0, Vec2::Y, Vec2::new(10.0, 0.0), None);
        assert!(cast.hit);
        assert_eq!(cast.distance, 5.0);
        assert_eq!(cast.normal, Vec2::Y);
        assert_eq!(cast.point, Vec2::new(10.0, 0.0));
    }

    // ==================== GroundInfo Tests ====================

    #[test]
    fn ground_info_default_not_detected() {
        let info = GroundInfo::default();
        assert!(!info.detected);
        assert!(!info.is_grounded(8.0, 2.0));
    }

    #[test]
    fn ground_info_is_grounded() {
        let mut info = GroundInfo::default();
        info.detected = true;
        info.distance = 5.0;

        // Within float height
        assert!(info.is_grounded(8.0, 2.0));

        // At float height + cling distance
        info.distance = 10.0;
        assert!(info.is_grounded(8.0, 2.0));

        // Beyond cling distance
        info.distance = 11.0;
        assert!(!info.is_grounded(8.0, 2.0));
    }

    #[test]
    fn ground_info_not_grounded_when_not_detected() {
        let mut info = GroundInfo::default();
        info.distance = 5.0; // Within range but not detected
        assert!(!info.is_grounded(8.0, 2.0));
    }

    #[test]
    fn ground_info_is_on_slope() {
        let mut info = GroundInfo::default();
        info.detected = true;

        info.slope_angle = 0.0;
        assert!(!info.is_on_slope());

        info.slope_angle = 0.05;
        assert!(!info.is_on_slope()); // Below threshold

        info.slope_angle = 0.5;
        assert!(info.is_on_slope());
    }

    #[test]
    fn ground_info_tangent() {
        let mut info = GroundInfo::default();

        // Flat ground (normal up)
        info.normal = Vec2::Y;
        assert!((info.tangent() - Vec2::X).length() < 0.001);

        // Wall (normal right)
        info.normal = Vec2::X;
        assert!((info.tangent() - Vec2::NEG_Y).length() < 0.001);
    }

    #[test]
    fn ground_info_height_error() {
        let mut info = GroundInfo::default();

        // Not detected = no error
        info.detected = false;
        assert_eq!(info.height_error(8.0), 0.0);

        // Too close = positive error (push up)
        info.detected = true;
        info.distance = 5.0;
        assert_eq!(info.height_error(8.0), 3.0);

        // Too far = negative error (pull down)
        info.distance = 10.0;
        assert_eq!(info.height_error(8.0), -2.0);

        // At target = no error
        info.distance = 8.0;
        assert_eq!(info.height_error(8.0), 0.0);
    }

    // ==================== WallInfo Tests ====================

    #[test]
    fn wall_info_default_no_walls() {
        let info = WallInfo::default();
        assert!(!info.left_detected);
        assert!(!info.right_detected);
        assert!(!info.any_wall());
    }

    #[test]
    fn wall_info_any_wall() {
        let mut info = WallInfo::default();

        info.left_detected = true;
        assert!(info.any_wall());

        info.left_detected = false;
        info.right_detected = true;
        assert!(info.any_wall());

        info.right_detected = false;
        assert!(!info.any_wall());
    }

    #[test]
    fn wall_info_is_blocked() {
        let mut info = WallInfo::default();
        info.left_detected = true;
        info.left_distance = 2.0;
        info.right_detected = true;
        info.right_distance = 5.0;

        // Moving left, blocked by close wall
        assert!(info.is_blocked(-1.0, 3.0));

        // Moving right, not blocked (wall is farther)
        assert!(!info.is_blocked(1.0, 3.0));

        // Moving right, blocked with larger threshold
        assert!(info.is_blocked(1.0, 6.0));
    }

    #[test]
    fn wall_info_wall_normal() {
        let mut info = WallInfo::default();
        info.left_detected = true;
        info.left_normal = Vec2::X;
        info.right_detected = true;
        info.right_normal = Vec2::NEG_X;

        assert_eq!(info.wall_normal(-1.0), Some(Vec2::X));
        assert_eq!(info.wall_normal(1.0), Some(Vec2::NEG_X));

        info.left_detected = false;
        assert_eq!(info.wall_normal(-1.0), None);
    }

    #[test]
    fn wall_info_zero_direction() {
        let mut info = WallInfo::default();
        info.left_detected = true;
        info.right_detected = true;

        // Zero direction should not match either side
        assert_eq!(info.wall_normal(0.0), None);
        assert!(!info.is_blocked(0.0, 1.0));
    }
}
