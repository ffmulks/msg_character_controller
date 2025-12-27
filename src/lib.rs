//! # `msg_character_controller`
//!
//! An advanced 2D floating rigidbody character controller with physics backend abstraction.
//!
//! This crate provides a responsive, tuneable character controller that:
//! - Floats above ground using a spring-damper system
//! - Uses raycasts for ground detection and slope handling
//! - Supports both walking (1D intent) and flying (2D intent) modes
//! - Handles stair stepping with multi-raycast detection
//! - Detects wall contact for advanced movement
//! - Abstracts physics backend for easy swapping (Rapier2D included)
//!
//! ## Architecture
//!
//! The controller uses a **floating rigibody** approach where:
//! 1. A dynamic rigidbody handles collisions normally
//! 2. Raycasts detect ground and compute desired float height
//! 3. A spring-damper system applies forces to maintain float height
//! 4. Movement is controlled via velocity/impulse on the horizontal axis
//!
//! ## Usage
//!
//! ```rust
//! use bevy::prelude::*;
//! use msg_character_controller::prelude::*;
//!
//! // Create controller components for a walking character
//! let controller = CharacterController::walking();
//! let config = ControllerConfig::player();
//! let intent = WalkIntent::default();
//!
//! // These can be spawned as a bundle with physics components
//! ```

use bevy::prelude::*;

pub mod backend;
pub mod config;
pub mod detection;
pub mod intent;
pub mod state;
pub mod systems;

#[cfg(feature = "rapier2d")]
pub mod rapier;

pub mod prelude {
    //! Convenient re-exports for common usage.

    pub use crate::backend::CharacterPhysicsBackend;
    pub use crate::config::{
        CharacterController, CharacterOrientation, ControllerConfig, StairConfig,
    };
    pub use crate::detection::{GroundInfo, SensorCast, WallInfo};
    pub use crate::intent::{FlyIntent, JumpRequest, WalkIntent};
    pub use crate::state::{Airborne, Grounded, TouchingWall};
    pub use crate::CharacterControllerPlugin;

    #[cfg(feature = "rapier2d")]
    pub use crate::rapier::Rapier2dBackend;
}

/// Main plugin for the character controller system.
///
/// This plugin is generic over a physics backend `B` which provides the actual
/// physics operations (raycasting, force application, etc.).
///
/// # Type Parameters
/// - `B`: The physics backend implementation (e.g., `Rapier2dBackend`)
///
/// # Examples
///
/// With Rapier2D backend:
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
pub struct CharacterControllerPlugin<B: backend::CharacterPhysicsBackend> {
    _marker: std::marker::PhantomData<B>,
}

impl<B: backend::CharacterPhysicsBackend> Default for CharacterControllerPlugin<B> {
    fn default() -> Self {
        Self {
            _marker: std::marker::PhantomData,
        }
    }
}

impl<B: backend::CharacterPhysicsBackend> Plugin for CharacterControllerPlugin<B> {
    fn build(&self, app: &mut App) {
        // Register core types
        app.register_type::<config::CharacterController>();
        app.register_type::<config::CharacterOrientation>();
        app.register_type::<config::ControllerConfig>();
        app.register_type::<config::StairConfig>();
        app.register_type::<intent::WalkIntent>();
        app.register_type::<intent::FlyIntent>();
        app.register_type::<intent::JumpRequest>();
        app.register_type::<state::Grounded>();
        app.register_type::<state::Airborne>();
        app.register_type::<state::TouchingWall>();
        app.register_type::<detection::GroundInfo>();
        app.register_type::<detection::WallInfo>();

        // Add the physics backend plugin
        app.add_plugins(B::plugin());

        // Add core systems in FixedUpdate for consistent physics behavior
        app.add_systems(
            FixedUpdate,
            (
                systems::update_ground_detection::<B>,
                systems::update_wall_detection::<B>,
                systems::apply_floating_spring::<B>,
                systems::apply_upright_torque::<B>,
                systems::apply_walk_movement::<B>,
                systems::apply_fly_movement::<B>,
                systems::apply_jump::<B>,
                systems::sync_state_markers,
            )
                .chain(),
        );

        // Reset jump requests at end of fixed update
        app.add_systems(FixedPostUpdate, systems::reset_jump_requests);
    }
}
