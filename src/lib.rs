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
//! ## System Order
//!
//! Systems run in clearly defined phases using [`CharacterControllerSet`]:
//!
//! 1. **Preparation** - Clear forces from previous frame
//! 2. **Sensors** - Collect ground/wall/ceiling data (run in parallel)
//! 3. **IntentEvaluation** - Read MovementIntent, set intent flags (requires current sensor data)
//! 4. **ForceAccumulation** - Spring, gravity, stair climb, upright torque
//! 5. **IntentApplication** - Apply jump, walk, fly based on intent
//! 6. **FinalApplication** - Apply accumulated forces to physics
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

// Systems are internal - they're added automatically by the plugin
pub(crate) mod systems;

#[cfg(feature = "rapier2d")]
pub mod rapier;

/// System sets for character controller phases.
///
/// These sets define the order of operations for the character controller:
///
/// 1. **Preparation** - Clear forces from previous frame
/// 2. **Sensors** - Collect ground/wall/ceiling data (systems can run in parallel)
/// 3. **IntentEvaluation** - Read MovementIntent, set intent flags (requires current sensor data)
/// 4. **ForceAccumulation** - Accumulate spring, gravity, stair climb, upright torque forces
/// 5. **IntentApplication** - Apply jump, walk, fly impulses based on intent
/// 6. **FinalApplication** - Apply accumulated forces to physics engine
///
/// # Usage
///
/// Systems within the same set can run in parallel if they don't depend on each other.
/// Use `.chain()` for groups that need sequential execution within a set.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CharacterControllerSet {
    /// Phase 1: Clear forces from previous frame.
    Preparation,
    /// Phase 2: Collect sensor data (ground, wall, ceiling detection).
    /// Systems in this phase can run in parallel.
    Sensors,
    /// Phase 3: Evaluate MovementIntent and set intent flags.
    /// Runs after Sensors so it has access to current frame's floor/grounded state.
    IntentEvaluation,
    /// Phase 4: Accumulate forces (spring, gravity, stair climb, upright torque).
    ForceAccumulation,
    /// Phase 5: Apply intent-based impulses (jump, walk, fly).
    IntentApplication,
    /// Phase 6: Apply accumulated forces to physics engine.
    FinalApplication,
}

pub mod prelude {
    //! Convenient re-exports for common usage.
    //!
    //! This module provides all the types you need to get started with the character controller:
    //!
    //! ```rust,no_run
    //! use bevy::prelude::*;
    //! use bevy_rapier2d::prelude::*;
    //! use msg_character_controller::prelude::*;
    //!
    //! fn spawn_character(mut commands: Commands) {
    //!     commands.spawn((
    //!         Transform::from_xyz(0.0, 100.0, 0.0),
    //!         CharacterController::new(),
    //!         ControllerConfig::player(),
    //!         MovementIntent::default(),
    //!         Rapier2dCharacterBundle::rotation_locked(),
    //!         Collider::capsule_y(8.0, 4.0),
    //!         GravityScale(0.0),
    //!     ));
    //! }
    //! ```

    pub use crate::CharacterControllerPlugin;
    pub use crate::CharacterControllerSet;
    pub use crate::backend::CharacterPhysicsBackend;
    pub use crate::collision::CollisionData;
    pub use crate::config::{CharacterController, ControllerConfig, JumpType, StairConfig};
    pub use crate::intent::{JumpRequest, MovementIntent};

    #[cfg(feature = "rapier2d")]
    pub use crate::rapier::{Rapier2dBackend, Rapier2dCharacterBundle};
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
        app.register_type::<config::ControllerConfig>();
        app.register_type::<config::StairConfig>();
        app.register_type::<config::JumpType>();
        app.register_type::<intent::MovementIntent>();

        // Add the physics backend plugin
        app.add_plugins(B::plugin());

        // Configure system set ordering
        // Phase order: Preparation -> Sensors -> IntentEvaluation -> ForceAccumulation -> IntentApplication -> FinalApplication
        // NOTE: Sensors must run BEFORE IntentEvaluation so that intent evaluation
        // has access to current frame's floor/grounded state. This ensures that
        // intends_upward_propulsion is correctly set before spring forces are calculated.
        app.configure_sets(
            FixedUpdate,
            (
                CharacterControllerSet::Preparation,
                CharacterControllerSet::Sensors,
                CharacterControllerSet::IntentEvaluation,
                CharacterControllerSet::ForceAccumulation,
                CharacterControllerSet::IntentApplication,
                CharacterControllerSet::FinalApplication,
            )
                .chain(),
        );

        // Phase 1: Preparation
        // Process jump state (edge detection), tick request timers, remove expired requests
        // These must run before sensors/intent evaluation
        app.add_systems(
            FixedUpdate,
            (
                systems::process_jump_state,
                systems::tick_jump_request_timers,
                systems::expire_jump_requests,
            )
                .chain()
                .in_set(CharacterControllerSet::Preparation),
        );

        // Phase 3: Intent Evaluation
        // First update timers, then update jump type, then evaluate MovementIntent
        // This runs AFTER sensors so it has access to current frame's floor/grounded state
        // update_jump_type must run before evaluate_intent so buffered jumps use correct type
        app.add_systems(
            FixedUpdate,
            (
                systems::update_timers,
                systems::update_jump_type,
                systems::evaluate_intent::<B>,
            )
                .chain()
                .in_set(CharacterControllerSet::IntentEvaluation),
        );

        // Phase 4: Force Accumulation
        // These systems accumulate forces based on sensor data and intent
        // Spring must run after stair climbing (which sets active_stair_height)
        // Gravity and upright torque can run in parallel with each other
        app.add_systems(
            FixedUpdate,
            (
                systems::accumulate_stair_climb_force::<B>,
                systems::accumulate_spring_force::<B>,
            )
                .chain()
                .in_set(CharacterControllerSet::ForceAccumulation),
        );
        app.add_systems(
            FixedUpdate,
            (
                systems::accumulate_gravity::<B>,
                systems::accumulate_upright_torque::<B>,
            )
                .in_set(CharacterControllerSet::ForceAccumulation),
        );

        // Phase 5: Intent Application
        // Apply impulses based on intent (fall gravity, jump, walk, fly)
        // Fall gravity runs first to check jump request before it's consumed
        // Jump runs second to consume the jump request
        // Walk and fly can run in parallel as they affect orthogonal axes
        app.add_systems(
            FixedUpdate,
            (systems::apply_fall_gravity::<B>, systems::apply_jump::<B>)
                .chain()
                .in_set(CharacterControllerSet::IntentApplication),
        );
        app.add_systems(
            FixedUpdate,
            (
                systems::apply_walk::<B>,
                systems::apply_fly::<B>,
                systems::apply_wall_clinging_dampening::<B>,
            )
                .in_set(CharacterControllerSet::IntentApplication),
        );
    }
}
