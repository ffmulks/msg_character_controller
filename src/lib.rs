//! # `msg_character_controller`
//!
//! An advanced 2D floating rigidbody character controller with physics backend abstraction.
//!
//! This crate provides a responsive, tuneable character controller that:
//! - Floats above ground using a spring-damper system
//! - Uses raycasts for ground detection and slope handling
//! - Horizontal movement via WalkIntent with slope handling when grounded
//! - Vertical propulsion via PropulsionIntent (jetpack/thrusters) with gravity compensation
//! - Distinct jump functionality with coyote time and input buffering
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
//! 4. Horizontal movement and vertical propulsion is controlled via MovementIntent
//! 5. Jumping is an impulse-based action that requires being grounded
//!
//! ## Usage
//!
//! ```rust
//! use bevy::prelude::*;
//! use msg_character_controller::prelude::*;
//!
//! // Create controller components
//! let controller = CharacterController::new();
//! let config = ControllerConfig::player();
//! let movement = MovementIntent::default();
//!
//! // These can be spawned as a bundle with physics components
//! ```

use bevy::prelude::*;

pub mod backend;
pub mod collision;
pub mod config;
pub mod intent;
pub mod systems;

#[cfg(feature = "rapier2d")]
pub mod rapier;

pub mod prelude {
    //! Convenient re-exports for common usage.

    pub use crate::CharacterControllerPlugin;
    pub use crate::backend::CharacterPhysicsBackend;
    pub use crate::collision::CollisionData;
    pub use crate::config::{
        CharacterController, CharacterOrientation, ControllerConfig, StairConfig,
    };
    pub use crate::intent::{JumpRequest, MovementIntent};

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
/// # Gravity Handling
/// Gravity is always applied internally by this plugin as a force. Set the desired
/// gravity vector on `CharacterController::gravity`. External application of gravity
/// is not supported - use `CharacterController::set_gravity()` to change the gravity
/// vector at runtime.
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

impl<B: backend::CharacterPhysicsBackend> CharacterControllerPlugin<B> {
    /// Create a new character controller plugin.
    pub fn new() -> Self {
        Self::default()
    }
}

impl<B: backend::CharacterPhysicsBackend> Plugin for CharacterControllerPlugin<B> {
    fn build(&self, app: &mut App) {
        // Register core types
        app.register_type::<config::CharacterController>();
        app.register_type::<config::CharacterOrientation>();
        app.register_type::<config::ControllerConfig>();
        app.register_type::<config::StairConfig>();
        app.register_type::<intent::MovementIntent>();

        // Add the physics backend plugin
        app.add_plugins(B::plugin());

        // Add core systems in FixedUpdate for consistent physics behavior
        // These must run before Rapier's StepSimulation to ensure forces are integrated
        app.add_systems(
            FixedUpdate,
            (
                systems::apply_floating_spring::<B>,
                systems::apply_gravity::<B>,
                systems::apply_upright_torque::<B>,
                systems::apply_movement::<B>,
                systems::apply_jump::<B>,
            )
                .chain()
                .before(bevy_rapier2d::plugin::PhysicsSet::StepSimulation),
        );
    }
}
