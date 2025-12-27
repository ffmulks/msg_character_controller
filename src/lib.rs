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
//! 4. Horizontal movement is controlled via WalkIntent
//! 5. Vertical propulsion is controlled via PropulsionIntent (with gravity boost for upward)
//! 6. Jumping is an impulse-based action that requires being grounded
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
//! let walk_intent = WalkIntent::default();
//! let propulsion = PropulsionIntent::default();
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
    pub use crate::config::{CharacterController, CharacterOrientation, ControllerConfig, StairConfig};
    pub use crate::detection::SensorCast;
    pub use crate::intent::{JumpRequest, PropulsionIntent, WalkIntent};
    pub use crate::state::{Airborne, Grounded, TouchingCeiling, TouchingWall};
    pub use crate::CharacterControllerPlugin;
    pub use crate::GravityMode;

    #[cfg(feature = "rapier2d")]
    pub use crate::rapier::Rapier2dBackend;
}

/// Configuration for how gravity is handled by the character controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GravityMode {
    /// The plugin applies gravity internally using the CharacterController.gravity field.
    /// Gravity is applied as a force each physics frame when the character is walking
    /// and not grounded.
    #[default]
    Internal,
    /// Gravity is applied externally by the user.
    /// The plugin will NOT apply gravity forces, but will still use the
    /// CharacterController.gravity field for:
    /// - Extra fall gravity calculation
    /// - Jump counter force
    /// - Cling force calculation
    External,
}

/// Main plugin for the character controller system.
///
/// This plugin is generic over a physics backend `B` which provides the actual
/// physics operations (raycasting, force application, etc.).
///
/// # Type Parameters
/// - `B`: The physics backend implementation (e.g., `Rapier2dBackend`)
///
/// # Configuration
/// - `gravity_mode`: Whether gravity is applied internally or externally
///
/// # Examples
///
/// With Rapier2D backend (internal gravity):
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
///
/// With external gravity:
/// ```rust,no_run
/// use bevy::prelude::*;
/// use bevy_rapier2d::prelude::*;
/// use msg_character_controller::prelude::*;
///
/// App::new()
///     .add_plugins(DefaultPlugins)
///     .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
///     .add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::with_external_gravity())
///     .run();
/// ```
pub struct CharacterControllerPlugin<B: backend::CharacterPhysicsBackend> {
    _marker: std::marker::PhantomData<B>,
    gravity_mode: GravityMode,
}

impl<B: backend::CharacterPhysicsBackend> Default for CharacterControllerPlugin<B> {
    fn default() -> Self {
        Self {
            _marker: std::marker::PhantomData,
            gravity_mode: GravityMode::Internal,
        }
    }
}

impl<B: backend::CharacterPhysicsBackend> CharacterControllerPlugin<B> {
    /// Create a plugin with internal gravity (default).
    /// The plugin will apply gravity forces to walking characters.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a plugin with external gravity.
    /// The user is responsible for applying gravity; the plugin will use
    /// CharacterController.gravity for calculations only.
    pub fn with_external_gravity() -> Self {
        Self {
            _marker: std::marker::PhantomData,
            gravity_mode: GravityMode::External,
        }
    }

    /// Create a plugin with the specified gravity mode.
    pub fn with_gravity_mode(gravity_mode: GravityMode) -> Self {
        Self {
            _marker: std::marker::PhantomData,
            gravity_mode,
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
        app.register_type::<intent::PropulsionIntent>();
        app.register_type::<intent::JumpRequest>();
        app.register_type::<state::Grounded>();
        app.register_type::<state::Airborne>();
        app.register_type::<state::TouchingWall>();
        app.register_type::<state::TouchingCeiling>();

        // Insert gravity mode as a resource
        app.insert_resource(GravityModeResource(self.gravity_mode));

        // Add the physics backend plugin
        app.add_plugins(B::plugin());

        // Add core systems in FixedUpdate for consistent physics behavior
        app.add_systems(
            FixedUpdate,
            (
                systems::apply_floating_spring::<B>,
                systems::apply_internal_gravity::<B>,
                systems::apply_upright_torque::<B>,
                systems::apply_walk_movement::<B>,
                systems::apply_propulsion::<B>,
                systems::apply_jump::<B>,
                systems::sync_state_markers,
            )
                .chain(),
        );

        // Reset jump requests at end of fixed update
        app.add_systems(FixedPostUpdate, systems::reset_jump_requests);
    }
}

/// Resource to store the gravity mode configuration.
#[derive(Resource, Debug, Clone, Copy)]
pub struct GravityModeResource(pub GravityMode);
