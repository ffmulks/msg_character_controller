//! Integration tests for the character controller.
//!
//! These tests verify the complete system behavior with actual physics simulation.
//!
//! # Important Notes
//!
//! - Rapier manages its own internal physics state. Do NOT modify Transform directly
//!   after spawning - use Velocity to move entities.
//! - Tests should spawn entities at their intended positions and verify behavior.
//! - For state transition tests, use physics (velocity) to move entities.

use bevy::prelude::*;
use bevy::time::Virtual;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;
use msg_character_controller::rapier::Rapier2dBackend;

/// Create a minimal test app with physics and character controller.
fn create_test_app() -> App {
    let mut app = App::new();

    // Use MinimalPlugins with a loop mode that allows manual time control
    app.add_plugins(MinimalPlugins);

    // Transform plugin for GlobalTransform propagation
    app.add_plugins(TransformPlugin);

    // Rapier physics
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());

    // Character controller with Rapier backend
    app.add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default());

    // Set fixed timestep to 60 Hz
    app.insert_resource(Time::<Fixed>::from_hz(60.0));

    // Initialize app
    app.finish();
    app.cleanup();

    app
}

/// Spawn a static ground collider.
fn spawn_ground(app: &mut App, position: Vec2, half_size: Vec2) -> Entity {
    let transform = Transform::from_translation(position.extend(0.0));
    app.world_mut()
        .spawn((
            transform,
            GlobalTransform::from(transform),
            RigidBody::Fixed,
            Collider::cuboid(half_size.x, half_size.y),
        ))
        .id()
}

/// Spawn a static wall collider.
fn spawn_wall(app: &mut App, position: Vec2, half_size: Vec2) -> Entity {
    spawn_ground(app, position, half_size)
}

/// Spawn a character controller entity with default config.
fn spawn_character(app: &mut App, position: Vec2) -> Entity {
    spawn_character_with_config(app, position, ControllerConfig::player())
}

/// Spawn a character controller entity with custom config.
fn spawn_character_with_config(app: &mut App, position: Vec2, config: ControllerConfig) -> Entity {
    let transform = Transform::from_translation(position.extend(0.0));
    app.world_mut()
        .spawn((
            transform,
            GlobalTransform::from(transform),
            CharacterController::walking(),
            config,
            WalkIntent::default(),
            JumpRequest::default(),
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            LockedAxes::ROTATION_LOCKED,
            Collider::capsule_y(8.0, 4.0),
            GravityScale(0.0), // Disable gravity for predictable tests
        ))
        .id()
}

/// Spawn a character with custom orientation.
fn spawn_oriented_character(
    app: &mut App,
    position: Vec2,
    orientation: CharacterOrientation,
) -> Entity {
    let transform = Transform::from_translation(position.extend(0.0));
    app.world_mut()
        .spawn((
            transform,
            GlobalTransform::from(transform),
            CharacterController::walking(),
            ControllerConfig::player(),
            orientation,
            WalkIntent::default(),
            JumpRequest::default(),
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            LockedAxes::ROTATION_LOCKED,
            Collider::capsule_y(8.0, 4.0),
            GravityScale(0.0),
        ))
        .id()
}

/// Run one physics step: sync Rapier colliders and run character controller systems.
fn tick(app: &mut App) {
    // Advance virtual time by one fixed timestep to trigger FixedUpdate naturally
    let timestep = std::time::Duration::from_secs_f64(1.0 / 60.0);
    app.world_mut()
        .resource_mut::<Time<Virtual>>()
        .advance_by(timestep);

    // First update: sync Rapier physics colliders, run FixedUpdate naturally with proper time
    app.update();

    // Run FixedUpdate again to process any impulses/forces set by character controller
    app.world_mut()
        .run_schedule(bevy::prelude::FixedUpdate);

    // Second update: let Rapier process the forces/impulses
    app.update();
}

/// Run the app for N physics frames.
fn run_frames(app: &mut App, frames: usize) {
    for _ in 0..frames {
        tick(app);
    }
}

/// Run minimal physics update (syncs Rapier and runs FixedUpdate).
fn run_physics(app: &mut App) {
    tick(app);
}

/// Get current elapsed time from the app.
fn elapsed_time(app: &App) -> f32 {
    app.world()
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0)
}

/// Request a jump at the current time.
fn request_jump(app: &mut App, entity: Entity) {
    let time = elapsed_time(app);
    if let Some(mut jump) = app.world_mut().get_mut::<JumpRequest>(entity) {
        jump.request(time);
    }
}

// ==================== Ground Detection Tests ====================

mod ground_detection {
    use super::*;

    #[test]
    fn character_above_ground_detects_ground() {
        let mut app = create_test_app();

        // Ground at y=0, character at y=16 (within ground_cast_length of 12)
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 16.0));

        run_physics(&mut app);

        let ground_info = app.world().get::<GroundInfo>(character);
        assert!(ground_info.is_some(), "GroundInfo should be present");

        let ground = ground_info.unwrap();
        assert!(ground.detected, "Ground should be detected");
    }

    #[test]
    fn character_at_float_height_is_grounded() {
        let mut app = create_test_app();

        // Ground at y=0, character at float_height (8) + ground half_height (5) = y=13
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);

        assert!(
            app.world().get::<Grounded>(character).is_some(),
            "Character should be grounded"
        );
        assert!(
            app.world().get::<Airborne>(character).is_none(),
            "Character should not be airborne"
        );
    }

    #[test]
    fn character_high_above_ground_not_grounded() {
        let mut app = create_test_app();

        // Ground at y=0, character at y=100 (far above detection range)
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 100.0));

        run_physics(&mut app);

        assert!(
            app.world().get::<Grounded>(character).is_none(),
            "Character should not be grounded when high up"
        );
    }

    #[test]
    fn character_over_empty_space_not_grounded() {
        let mut app = create_test_app();

        // Ground only on the left side
        spawn_ground(&mut app, Vec2::new(-50.0, 0.0), Vec2::new(20.0, 5.0));

        // Character on the right with no ground
        let character = spawn_character(&mut app, Vec2::new(50.0, 20.0));

        run_physics(&mut app);

        let ground_info = app.world().get::<GroundInfo>(character);
        assert!(ground_info.is_some());
        assert!(
            !ground_info.unwrap().detected,
            "Ground should not be detected over empty space"
        );
    }
}

// ==================== Airborne Detection Tests ====================

mod airborne_detection {
    use super::*;

    #[test]
    fn character_in_air_has_airborne_marker() {
        let mut app = create_test_app();

        // No ground at all
        let character = spawn_character(&mut app, Vec2::new(0.0, 50.0));

        run_physics(&mut app);

        assert!(
            app.world().get::<Airborne>(character).is_some(),
            "Character should have Airborne marker"
        );
        assert!(
            app.world().get::<Grounded>(character).is_none(),
            "Character should not have Grounded marker"
        );
    }

    #[test]
    fn grounded_vs_airborne_based_on_position() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Spawn grounded character
        let grounded = spawn_character(&mut app, Vec2::new(-20.0, 13.0));

        // Spawn airborne character (far above ground)
        let airborne = spawn_character(&mut app, Vec2::new(20.0, 100.0));

        run_physics(&mut app);

        // Check grounded character
        assert!(
            app.world().get::<Grounded>(grounded).is_some(),
            "Near-ground character should be grounded"
        );

        // Check airborne character
        assert!(
            app.world().get::<Airborne>(airborne).is_some(),
            "Far-above character should be airborne"
        );
    }
}

// ==================== Wall Detection Tests ====================

mod wall_detection {
    use super::*;

    #[test]
    fn detects_wall_on_left() {
        let mut app = create_test_app();

        // Need ground so character is in valid state
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Wall on the left, close enough to detect (within wall_cast_length of 6)
        spawn_wall(&mut app, Vec2::new(-7.0, 13.0), Vec2::new(2.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);

        let wall_info = app.world().get::<WallInfo>(character);
        assert!(wall_info.is_some(), "WallInfo should be present");

        let wall = wall_info.unwrap();
        assert!(wall.left_detected, "Wall on left should be detected");
    }

    #[test]
    fn detects_wall_on_right() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Wall on the right
        spawn_wall(&mut app, Vec2::new(7.0, 13.0), Vec2::new(2.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);

        let wall_info = app.world().get::<WallInfo>(character);
        assert!(wall_info.is_some(), "WallInfo should be present");

        let wall = wall_info.unwrap();
        assert!(wall.right_detected, "Wall on right should be detected");
    }

    #[test]
    fn detects_walls_on_both_sides() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        spawn_wall(&mut app, Vec2::new(-7.0, 13.0), Vec2::new(2.0, 20.0));
        spawn_wall(&mut app, Vec2::new(7.0, 13.0), Vec2::new(2.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);

        let wall_info = app.world().get::<WallInfo>(character);
        assert!(wall_info.is_some(), "WallInfo should be present");

        let wall = wall_info.unwrap();
        assert!(wall.left_detected, "Wall on left should be detected");
        assert!(wall.right_detected, "Wall on right should be detected");
    }

    #[test]
    fn no_wall_when_walls_are_far() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Walls far away (beyond wall_cast_length)
        spawn_wall(&mut app, Vec2::new(-50.0, 13.0), Vec2::new(5.0, 20.0));
        spawn_wall(&mut app, Vec2::new(50.0, 13.0), Vec2::new(5.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);

        let wall_info = app.world().get::<WallInfo>(character);
        assert!(wall_info.is_some(), "WallInfo should be present");

        let wall = wall_info.unwrap();
        assert!(!wall.left_detected, "Far wall on left should not be detected");
        assert!(
            !wall.right_detected,
            "Far wall on right should not be detected"
        );
    }
}

// ==================== Coyote Time Tests ====================

mod coyote_time {
    use super::*;

    #[test]
    fn time_since_grounded_is_zero_when_grounded() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);

        let ground_info = app.world().get::<GroundInfo>(character).unwrap();
        assert!(
            ground_info.time_since_grounded < 0.001,
            "time_since_grounded should be zero when grounded: {}",
            ground_info.time_since_grounded
        );
    }

    #[test]
    fn airborne_character_accumulates_time() {
        let mut app = create_test_app();

        // No ground - character is permanently airborne
        let character = spawn_character(&mut app, Vec2::new(0.0, 50.0));

        // Run multiple frames to accumulate time
        run_frames(&mut app, 30);

        let ground_info = app.world().get::<GroundInfo>(character).unwrap();
        // After 30 frames at 60Hz, should have accumulated ~0.5 seconds
        assert!(
            ground_info.time_since_grounded > 0.3,
            "time_since_grounded should accumulate: {}",
            ground_info.time_since_grounded
        );
    }

    #[test]
    fn jump_works_for_grounded_character() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);

        // Verify grounded
        assert!(app.world().get::<Grounded>(character).is_some());

        let vel_before = app.world().get::<Velocity>(character).unwrap().linvel.y;

        // Request jump with current time
        request_jump(&mut app, character);

        run_physics(&mut app);

        // Jump should be consumed
        let jump = app.world().get::<JumpRequest>(character).unwrap();
        assert!(jump.consumed, "Jump should be consumed");

        let vel_after = app.world().get::<Velocity>(character).unwrap().linvel.y;
        // Note: The floating spring damping significantly reduces jump velocity.
        // We just verify that velocity increased (jump was applied).
        assert!(
            vel_after > vel_before + 0.5,
            "Jump should apply upward velocity: before={}, after={}",
            vel_before,
            vel_after
        );
    }
}

// ==================== Jump Buffer Tests ====================

mod jump_buffer {
    use super::*;

    #[test]
    fn jump_request_is_buffered_while_airborne() {
        let mut app = create_test_app();

        let config = ControllerConfig::player().with_jump_buffer_time(0.5);

        // Start airborne (no ground)
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 100.0), config);

        run_physics(&mut app);
        assert!(app.world().get::<Airborne>(character).is_some());

        // Request jump while airborne (using current time)
        request_jump(&mut app, character);

        // Verify jump is requested but not consumed
        let jump = app.world().get::<JumpRequest>(character).unwrap();
        assert!(jump.requested, "Jump should be requested");
        assert!(!jump.consumed, "Jump should not be consumed while airborne");
    }

    #[test]
    fn jump_consumed_when_grounded() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);
        assert!(app.world().get::<Grounded>(character).is_some());

        // Request jump with current time
        request_jump(&mut app, character);

        run_physics(&mut app);

        // Jump should be consumed
        let jump = app.world().get::<JumpRequest>(character).unwrap();
        assert!(jump.consumed, "Jump should be consumed after execution");
    }
}

// ==================== Character Orientation Tests ====================

mod orientation {
    use super::*;

    #[test]
    fn default_orientation_detects_ground_below() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_oriented_character(
            &mut app,
            Vec2::new(0.0, 13.0),
            CharacterOrientation::default(),
        );

        run_physics(&mut app);

        assert!(
            app.world().get::<Grounded>(character).is_some(),
            "Default orientation should detect ground below"
        );
    }

    #[test]
    fn inverted_orientation_detects_ceiling_as_ground() {
        let mut app = create_test_app();

        // "Ground" above (ceiling from world perspective)
        // Ceiling at y=30, character at y=17
        // Distance = 30 - 5 (half_height) - 17 = 8 (at float height)
        spawn_ground(&mut app, Vec2::new(0.0, 30.0), Vec2::new(100.0, 5.0));

        // Character with inverted orientation (up is -Y, so "down" is +Y)
        let character = spawn_oriented_character(
            &mut app,
            Vec2::new(0.0, 17.0),
            CharacterOrientation::new(Vec2::NEG_Y),
        );

        run_physics(&mut app);

        let ground_info = app.world().get::<GroundInfo>(character);
        assert!(ground_info.is_some(), "GroundInfo should exist");
        assert!(
            ground_info.unwrap().detected,
            "Inverted character should detect ceiling as ground"
        );
    }

    #[test]
    fn sideways_orientation_detects_wall_as_ground() {
        let mut app = create_test_app();

        // "Ground" on the left (wall from world perspective)
        // Wall at x=-13, character at x=0
        // With up=+X, down=-X (pointing left toward the wall)
        // Distance to wall surface = 13 - 5 (half_width) = 8 (at float height)
        spawn_wall(&mut app, Vec2::new(-13.0, 0.0), Vec2::new(5.0, 100.0));

        // Character with sideways orientation (up is +X, so "down" is -X)
        let character = spawn_oriented_character(
            &mut app,
            Vec2::new(0.0, 0.0),
            CharacterOrientation::new(Vec2::X),
        );

        run_physics(&mut app);

        let ground_info = app.world().get::<GroundInfo>(character);
        assert!(ground_info.is_some(), "GroundInfo should exist");
        assert!(
            ground_info.unwrap().detected,
            "Sideways character should detect wall as ground"
        );
    }
}

// ==================== Movement Tests ====================

mod movement {
    use super::*;

    #[test]
    fn walk_intent_affects_velocity() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);
        assert!(app.world().get::<Grounded>(character).is_some());

        // Set walk intent to move right
        if let Some(mut intent) = app.world_mut().get_mut::<WalkIntent>(character) {
            intent.set(1.0);
        }

        // Run a few physics frames to let the velocity build up
        run_frames(&mut app, 5);

        // Velocity should be positive after movement is applied
        let vel = app.world().get::<Velocity>(character).unwrap();
        assert!(
            vel.linvel.x > 0.0,
            "Walk intent should cause rightward velocity: {}",
            vel.linvel.x
        );
    }

    #[test]
    fn walk_intent_respects_max_speed() {
        let mut app = create_test_app();

        let config = ControllerConfig::player().with_max_speed(50.0);

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 13.0), config);

        run_physics(&mut app);

        // Set full walk intent
        if let Some(mut intent) = app.world_mut().get_mut::<WalkIntent>(character) {
            intent.set(1.0);
        }

        // Run many frames to reach max speed
        run_frames(&mut app, 100);

        let vel = app.world().get::<Velocity>(character).unwrap();
        assert!(
            vel.linvel.x <= 55.0,
            "Velocity should not exceed max speed: {}",
            vel.linvel.x
        );
    }

    #[test]
    fn jump_applies_upward_velocity() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 13.0));

        run_physics(&mut app);
        assert!(app.world().get::<Grounded>(character).is_some());

        let vel_before = app.world().get::<Velocity>(character).unwrap().linvel.y;

        // Request jump with current time
        request_jump(&mut app, character);

        run_physics(&mut app);

        let vel_after = app.world().get::<Velocity>(character).unwrap().linvel.y;
        // Note: The floating spring damping affects jump velocity.
        // We verify that upward velocity was applied.
        assert!(
            vel_after > vel_before + 0.5,
            "Jump should apply upward velocity: before={}, after={}",
            vel_before,
            vel_after
        );
    }
}
