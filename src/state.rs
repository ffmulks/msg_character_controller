//! State marker components.
//!
//! These components indicate the current physical state of a character controller.
//! They are automatically added/removed by the controller systems based on
//! physics detection results.

use bevy::prelude::*;

/// Marker component indicating the character is grounded.
///
/// Added automatically when the ground detection raycast finds valid ground
/// within the cling distance. Removed when the character becomes airborne.
///
/// This is a marker component - it has no data, just indicates state.
///
/// # Example
///
/// ```rust
/// use bevy::prelude::*;
/// use msg_character_controller::prelude::*;
///
/// // Grounded is a marker component - just use it in queries
/// fn check_grounded(grounded: Option<&Grounded>) -> bool {
///     grounded.is_some()
/// }
/// ```
#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct Grounded;

/// Marker component indicating the character is airborne.
///
/// Added automatically when the character leaves ground contact.
/// Mutually exclusive with [`Grounded`].
#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct Airborne;

/// Marker component indicating the character is touching a wall.
///
/// Added when wall detection raycasts find a surface that is too steep
/// to walk on (above `max_slope_angle`).
///
/// Contains the direction to the wall (normalized).
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct TouchingWall {
    /// Direction from character center to wall (normalized).
    pub direction: Vec2,
    /// Normal of the wall surface.
    pub normal: Vec2,
}

impl Default for TouchingWall {
    fn default() -> Self {
        Self {
            direction: Vec2::X,
            normal: Vec2::NEG_X,
        }
    }
}

impl TouchingWall {
    /// Create a new wall touch state.
    pub fn new(direction: Vec2, normal: Vec2) -> Self {
        Self { direction, normal }
    }

    /// Check if the wall is on the left side.
    pub fn is_left(&self) -> bool {
        self.direction.x < 0.0
    }

    /// Check if the wall is on the right side.
    pub fn is_right(&self) -> bool {
        self.direction.x > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grounded_is_default() {
        let grounded = Grounded::default();
        // Marker component, just verify it can be created
        let _ = grounded;
    }

    #[test]
    fn airborne_is_default() {
        let airborne = Airborne::default();
        let _ = airborne;
    }

    #[test]
    fn touching_wall_new() {
        let wall = TouchingWall::new(Vec2::NEG_X, Vec2::X);
        assert_eq!(wall.direction, Vec2::NEG_X);
        assert_eq!(wall.normal, Vec2::X);
    }

    #[test]
    fn touching_wall_is_left() {
        let wall = TouchingWall::new(Vec2::NEG_X, Vec2::X);
        assert!(wall.is_left());
        assert!(!wall.is_right());
    }

    #[test]
    fn touching_wall_is_right() {
        let wall = TouchingWall::new(Vec2::X, Vec2::NEG_X);
        assert!(wall.is_right());
        assert!(!wall.is_left());
    }

    #[test]
    fn touching_wall_diagonal() {
        // Diagonal wall contact
        let wall = TouchingWall::new(Vec2::new(-1.0, 0.5), Vec2::new(0.7, -0.7));
        assert!(wall.is_left()); // x < 0
        assert!(!wall.is_right());
    }
}
