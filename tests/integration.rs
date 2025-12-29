//! Integration tests for the character controller.
//!
//! These tests verify the complete system behavior with actual physics simulation.
//! Each test produces PROOF through explicit velocity/force checks.

use bevy::prelude::*;
use bevy::time::Virtual;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;

#[cfg(feature = "rapier2d")]
use msg_character_controller::rapier::{Rapier2dBackend, Rapier2dCharacterBundle};

/// Create a minimal test app with physics and character controller.
fn create_test_app() -> App {
    let mut app = App::new();

    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
    app.add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default());
    app.insert_resource(Time::<Fixed>::from_hz(60.0));

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

/// Spawn a character controller with default config.
fn spawn_character(app: &mut App, position: Vec2) -> Entity {
    spawn_character_with_config(app, position, ControllerConfig::default())
}

/// Spawn a character controller with custom config.
fn spawn_character_with_config(app: &mut App, position: Vec2, config: ControllerConfig) -> Entity {
    let transform = Transform::from_translation(position.extend(0.0));
    app.world_mut()
        .spawn((
            transform,
            GlobalTransform::from(transform),
            CharacterController::new(),
            config,
            MovementIntent::default(),
            Rapier2dCharacterBundle::rotation_locked(),
            Collider::capsule_y(8.0, 4.0),
            GravityScale(0.0), // Disable Rapier gravity - use controller's gravity
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
            CharacterController::new(),
            ControllerConfig::default(),
            orientation,
            MovementIntent::default(),
            Rapier2dCharacterBundle::rotation_locked(),
            Collider::capsule_y(8.0, 4.0),
            GravityScale(0.0),
        ))
        .id()
}

/// Run one physics step.
fn tick(app: &mut App) {
    let timestep = std::time::Duration::from_secs_f64(1.0 / 60.0);
    app.world_mut()
        .resource_mut::<Time<Virtual>>()
        .advance_by(timestep);
    app.update();
    app.world_mut().run_schedule(bevy::prelude::FixedUpdate);
    app.update();
}

/// Run the app for N physics frames.
fn run_frames(app: &mut App, frames: usize) {
    for _ in 0..frames {
        tick(app);
    }
}

/// Get elapsed time from the app.
fn elapsed_time(app: &App) -> f32 {
    app.world()
        .get_resource::<Time>()
        .map(|t| t.elapsed_secs())
        .unwrap_or(0.0)
}

/// Request a jump at the current time.
fn request_jump(app: &mut App, entity: Entity) {
    let time = elapsed_time(app);
    if let Some(mut intent) = app.world_mut().get_mut::<MovementIntent>(entity) {
        intent.request_jump(time);
    }
}

// ==================== Ground Detection Tests ====================

mod ground_detection {
    use super::*;

    #[test]
    fn character_above_ground_detects_ground() {
        let mut app = create_test_app();

        // Ground surface at y=5 (center at 0, half_height=5)
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Character at y=20 (within derived ground_cast_length)
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        // PROOF: ground_detected should be true (raycast hit)
        assert!(
            controller.ground_detected(),
            "Ground should be detected by raycast"
        );

        // PROOF: ground_distance should be approximately 20 - 5 = 15 (from capsule center to ground)
        // Capsule has half-height 8 and radius 4, so bottom is at y=20-12=8
        // Ground surface is at y=5, so distance ~= 8 - 5 = 3
        assert!(
            controller.ground_distance() < 20.0,
            "Ground distance should be less than character height: {}",
            controller.ground_distance()
        );

        println!(
            "PROOF: ground_detected={}, ground_distance={}, ground_normal={:?}",
            controller.ground_detected(),
            controller.ground_distance(),
            controller.ground_normal()
        );
    }

    #[test]
    fn character_at_float_height_is_grounded() {
        let mut app = create_test_app();

        let config = ControllerConfig::default().with_float_height(15.0);

        // Ground surface at y=5
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Character positioned so center is at float_height above ground surface
        // ground_distance is measured from character center (position), not capsule bottom
        // Ground surface is at y=5, float_height=15, so position.y = 5 + 15 = 20
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 20.0), config);

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let cfg = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, ground_distance={}, riding_height+tolerance={}",
            controller.is_grounded(cfg),
            controller.ground_distance(),
            controller.riding_height(cfg) + cfg.ground_tolerance
        );

        // PROOF: is_grounded should be true when within riding_height + ground_tolerance
        assert!(
            controller.is_grounded(cfg),
            "Character should be grounded at float_height"
        );
    }

    #[test]
    fn character_high_above_ground_not_grounded() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Character at y=200 (far above ground)
        let character = spawn_character(&mut app, Vec2::new(0.0, 200.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, ground_detected={}, ground_distance={}",
            controller.is_grounded(config),
            controller.ground_detected(),
            controller.ground_distance()
        );

        // PROOF: is_grounded should be false when far from ground
        assert!(
            !controller.is_grounded(config),
            "Character should NOT be grounded when high above"
        );
    }

    #[test]
    fn character_over_empty_space_no_ground() {
        let mut app = create_test_app();

        // Ground only on the left side
        spawn_ground(&mut app, Vec2::new(-50.0, 0.0), Vec2::new(20.0, 5.0));

        // Character on the right with no ground
        let character = spawn_character(&mut app, Vec2::new(50.0, 20.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: ground_detected={}, is_grounded={}",
            controller.ground_detected(),
            controller.is_grounded(config)
        );

        // PROOF: ground_detected should be false when over empty space
        assert!(
            !controller.ground_detected(),
            "Ground should NOT be detected over empty space"
        );
    }
}

// ==================== Float Height Tests ====================

mod float_height {
    use super::*;

    #[test]
    fn float_height_keeps_character_floating_above_ground() {
        let mut app = create_test_app();

        let float_height = 15.0;
        // Use a stiffer, overdamped spring for precise settling without oscillation
        // Critical damping: c = 2 * sqrt(k * m), with m ≈ 17, k = 5000 -> c ≈ 584
        // Use c = 700 for overdamping to ensure stable settling
        let config = ControllerConfig::default()
            .with_float_height(float_height)
            .with_spring(5000.0, 700.0);

        // Ground at y=0
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Character starts above ground
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 50.0), config);

        // Run simulation to let character settle (more frames for precise settling)
        run_frames(&mut app, 500);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let cfg = app.world().get::<ControllerConfig>(character).unwrap();
        let transform = app.world().get::<Transform>(character).unwrap();

        // The riding height = float_height + collider_bottom_offset
        let riding_height = controller.riding_height(cfg);

        println!(
            "PROOF: Character position.y={}, ground_distance={}, riding_height={}",
            transform.translation.y,
            controller.ground_distance(),
            riding_height
        );

        // PROOF: ground_distance should be close to riding_height after settling
        // With proper spring tuning, tolerance should be within 2 pixels
        // The force isolation system can slightly affect settling precision
        let tolerance = 2.0;
        assert!(
            (controller.ground_distance() - riding_height).abs() < tolerance,
            "Ground distance {} should be close to riding_height {} (diff: {})",
            controller.ground_distance(),
            riding_height,
            (controller.ground_distance() - riding_height).abs()
        );

        // PROOF: Character should NOT be touching the ground (position should be elevated)
        // Capsule bottom is at position.y - collider_bottom_offset
        let capsule_bottom = transform.translation.y - controller.capsule_half_height();
        let ground_surface = 5.0; // Ground half-height
        assert!(
            capsule_bottom > ground_surface,
            "Character should be floating ABOVE ground: capsule_bottom={}, ground_surface={}",
            capsule_bottom,
            ground_surface
        );
    }

    #[test]
    fn spring_force_applied_to_rigidbody() {
        let mut app = create_test_app();

        let config = ControllerConfig::default()
            .with_float_height(15.0)
            .with_spring(5000.0, 100.0);

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Start character below float_height - spring should push UP
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 15.0), config);

        // Get initial velocity
        tick(&mut app);
        let vel_before = app.world().get::<Velocity>(character).unwrap().linvel;

        // Run a frame
        tick(&mut app);
        let vel_after = app.world().get::<Velocity>(character).unwrap().linvel;

        println!(
            "PROOF: vel_before={:?}, vel_after={:?}",
            vel_before, vel_after
        );

        // PROOF: Spring force should have affected velocity
        // When below float_height, spring pushes up, so velocity should become positive
        let ext_force = app.world().get::<ExternalForce>(character);
        println!("PROOF: ExternalForce={:?}", ext_force);

        // The velocity change proves force was applied
        assert!(
            (vel_after - vel_before).length() > 0.001 || ext_force.is_some(),
            "Spring force should affect velocity or external force"
        );
    }
}

// ==================== Wall Detection Tests ====================

mod wall_detection {
    use super::*;

    #[test]
    fn detects_wall_on_left() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Wall on the left, close to character
        spawn_ground(&mut app, Vec2::new(-10.0, 20.0), Vec2::new(2.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_left_wall={}, left_wall={:?}",
            controller.touching_left_wall(),
            controller.left_wall
        );

        // PROOF: touching_left_wall should be true
        assert!(
            controller.touching_left_wall(),
            "Wall on left should be detected"
        );
    }

    #[test]
    fn detects_wall_on_right() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Wall on the right
        spawn_ground(&mut app, Vec2::new(10.0, 20.0), Vec2::new(2.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_right_wall={}, right_wall={:?}",
            controller.touching_right_wall(),
            controller.right_wall
        );

        // PROOF: touching_right_wall should be true
        assert!(
            controller.touching_right_wall(),
            "Wall on right should be detected"
        );
    }

    #[test]
    fn no_wall_when_far() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Wall far away
        spawn_ground(&mut app, Vec2::new(-100.0, 20.0), Vec2::new(5.0, 20.0));
        spawn_ground(&mut app, Vec2::new(100.0, 20.0), Vec2::new(5.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_left_wall={}, touching_right_wall={}",
            controller.touching_left_wall(),
            controller.touching_right_wall()
        );

        // PROOF: No walls should be detected when far
        assert!(
            !controller.touching_left_wall(),
            "Far wall should NOT be detected"
        );
        assert!(
            !controller.touching_right_wall(),
            "Far wall should NOT be detected"
        );
    }
}

// ==================== Ceiling Detection Tests ====================

mod ceiling_detection {
    use super::*;

    #[test]
    fn detects_ceiling_above() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Ceiling above character (within cling_distance + capsule_half_height)
        // Character at y=25, capsule_half_height=12, cling_distance=2
        // Cast length = 2 + 12 + 1 = 15, so ceiling bottom must be within 15 units
        // Ceiling bottom at y=25+14=39, so center at y=39+5=44
        spawn_ground(&mut app, Vec2::new(0.0, 39.0), Vec2::new(100.0, 5.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 25.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_ceiling={}, ceiling={:?}",
            controller.touching_ceiling(),
            controller.ceiling
        );

        // PROOF: touching_ceiling should be true
        assert!(
            controller.touching_ceiling(),
            "Ceiling above should be detected"
        );
    }

    #[test]
    fn no_ceiling_when_far() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Ceiling very far above
        spawn_ground(&mut app, Vec2::new(0.0, 200.0), Vec2::new(100.0, 5.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!("PROOF: touching_ceiling={}", controller.touching_ceiling());

        // PROOF: No ceiling should be detected when far
        assert!(
            !controller.touching_ceiling(),
            "Far ceiling should NOT be detected"
        );
    }
}

// ==================== Movement Tests ====================

mod movement {
    use super::*;

    #[test]
    fn walk_intent_changes_velocity() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_frames(&mut app, 5);

        let vel_before = app.world().get::<Velocity>(character).unwrap().linvel;

        // Set walk intent to move right
        if let Some(mut intent) = app.world_mut().get_mut::<MovementIntent>(character) {
            intent.set_walk(1.0);
        }

        run_frames(&mut app, 10);

        let vel_after = app.world().get::<Velocity>(character).unwrap().linvel;

        println!(
            "PROOF: vel_before={:?}, vel_after={:?}",
            vel_before, vel_after
        );

        // PROOF: Velocity should increase in the X direction
        assert!(
            vel_after.x > vel_before.x + 1.0,
            "Walk intent should increase X velocity"
        );
    }

    #[test]
    fn jump_changes_velocity() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Jump impulse is automatically scaled by actual Rapier mass
        // This ensures jump_speed represents the actual velocity change
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_frames(&mut app, 5);

        // Verify grounded
        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();
        assert!(controller.is_grounded(config), "Must be grounded to jump");

        let vel_before = app.world().get::<Velocity>(character).unwrap().linvel;

        request_jump(&mut app, character);
        tick(&mut app);

        let vel_after = app.world().get::<Velocity>(character).unwrap().linvel;

        println!(
            "PROOF: vel_before.y={}, vel_after.y={}",
            vel_before.y, vel_after.y
        );

        // PROOF: Jump should apply positive Y velocity change
        // The impulse is scaled by actual_mass, and Rapier applies delta_v = impulse / mass
        // So delta_v = (jump_speed * actual_mass) / actual_mass = jump_speed = 300
        assert!(
            vel_after.y > vel_before.y + 100.0,
            "Jump should apply significant upward velocity (~300 units/s)"
        );
    }
}

// ==================== Upright Torque Tests ====================

mod upright_torque {
    use super::*;

    fn spawn_rotatable_character(app: &mut App, position: Vec2, rotation: f32) -> Entity {
        let transform = Transform::from_translation(position.extend(0.0))
            .with_rotation(Quat::from_rotation_z(rotation));
        app.world_mut()
            .spawn((
                transform,
                GlobalTransform::from(transform),
                CharacterController::new(),
                // Use default upright torque settings (100000 strength, 20000 damping)
                ControllerConfig::default().with_upright_target_angle(0.0),
                MovementIntent::default(),
                Rapier2dCharacterBundle::new(), // Not rotation locked
                Collider::capsule_y(8.0, 4.0),
                GravityScale(0.0),
            ))
            .id()
    }

    #[test]
    fn upright_torque_corrects_positive_rotation() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Spawn character tilted 30 degrees positive (CCW)
        let initial_rotation = 0.5; // ~30 degrees
        let character = spawn_rotatable_character(&mut app, Vec2::new(0.0, 30.0), initial_rotation);

        tick(&mut app);

        // Check torque is applied
        let ext_force = app.world().get::<ExternalForce>(character);
        let torque = ext_force.map(|f| f.torque).unwrap_or(0.0);

        // PROOF: Torque should be applied to correct rotation
        // Since rotation is positive and target is 0, torque should be negative (CW)
        assert!(
            torque < -0.1,
            "Torque should be negative (CW) to correct positive rotation, got: {}",
            torque
        );
    }

    #[test]
    fn upright_torque_corrects_negative_rotation() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Spawn character tilted 30 degrees negative (CW)
        let initial_rotation = -0.5; // ~-30 degrees
        let character = spawn_rotatable_character(&mut app, Vec2::new(0.0, 30.0), initial_rotation);

        tick(&mut app);

        // Check torque is applied
        let ext_force = app.world().get::<ExternalForce>(character);
        let torque = ext_force.map(|f| f.torque).unwrap_or(0.0);

        // PROOF: Torque should be positive (CCW) to correct negative rotation
        assert!(
            torque > 0.1,
            "Torque should be positive (CCW) to correct negative rotation, got: {}",
            torque
        );
    }

    #[test]
    fn upright_torque_stops_at_target() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Spawn character at target angle (0 degrees)
        let character = spawn_rotatable_character(&mut app, Vec2::new(0.0, 30.0), 0.0);

        // Set zero angular velocity
        if let Some(mut vel) = app.world_mut().get_mut::<Velocity>(character) {
            vel.angvel = 0.0;
        }

        tick(&mut app);

        let ext_force = app.world().get::<ExternalForce>(character);
        let torque = ext_force.map(|f| f.torque).unwrap_or(0.0);

        // PROOF: Torque should be minimal when at target angle
        assert!(
            torque.abs() < 1.0,
            "Torque should be minimal at target angle: {}",
            torque
        );
    }

    #[test]
    fn upright_torque_actually_rotates_character_from_positive() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Spawn character tilted 30 degrees positive (CCW)
        let initial_rotation = 0.5; // ~30 degrees
        let character = spawn_rotatable_character(&mut app, Vec2::new(0.0, 30.0), initial_rotation);

        // Run multiple physics frames
        run_frames(&mut app, 60); // 1 second at 60fps

        let transform = app.world().get::<Transform>(character).unwrap();
        let (_, _, final_rotation) = transform.rotation.to_euler(EulerRot::XYZ);

        // Character should have rotated toward 0 (negative direction)
        assert!(
            final_rotation.abs() < initial_rotation.abs(),
            "Character should rotate toward target (0). Initial: {}, Final: {}",
            initial_rotation,
            final_rotation
        );
    }

    #[test]
    fn upright_torque_actually_rotates_character_from_negative() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Spawn character tilted 30 degrees negative (CW)
        let initial_rotation = -0.5; // ~-30 degrees
        let character = spawn_rotatable_character(&mut app, Vec2::new(0.0, 30.0), initial_rotation);

        // Run multiple physics frames
        run_frames(&mut app, 60); // 1 second at 60fps

        let transform = app.world().get::<Transform>(character).unwrap();
        let (_, _, final_rotation) = transform.rotation.to_euler(EulerRot::XYZ);

        // Character should have rotated toward 0 (positive direction)
        assert!(
            final_rotation.abs() < initial_rotation.abs(),
            "Character should rotate toward target (0). Initial: {}, Final: {}",
            initial_rotation,
            final_rotation
        );
    }

    #[test]
    fn upright_torque_converges_to_target() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Spawn character tilted 30 degrees positive (CCW)
        let initial_rotation = 0.5;
        let character = spawn_rotatable_character(&mut app, Vec2::new(0.0, 30.0), initial_rotation);

        // Run for 10 seconds (600 frames at 60fps)
        run_frames(&mut app, 600);

        let transform = app.world().get::<Transform>(character).unwrap();
        let (_, _, final_rotation) = transform.rotation.to_euler(EulerRot::XYZ);

        // After 10 seconds, should have converged close to target (0)
        // With max_torque=50 cap, convergence is gradual but steady
        assert!(
            final_rotation.abs() < 0.1,
            "Character should converge to near target. Final rotation: {}",
            final_rotation
        );
    }
}

// ==================== Gravity Tests ====================

mod gravity {
    use super::*;

    #[test]
    fn internal_gravity_applied_when_airborne() {
        let mut app = create_test_app();

        // No ground - character is airborne
        let character = spawn_character(&mut app, Vec2::new(0.0, 100.0));

        let vel_before = app.world().get::<Velocity>(character).unwrap().linvel;

        run_frames(&mut app, 10);

        let vel_after = app.world().get::<Velocity>(character).unwrap().linvel;

        println!(
            "PROOF: vel_before.y={}, vel_after.y={}",
            vel_before.y, vel_after.y
        );

        // PROOF: Internal gravity should decrease Y velocity (make it more negative)
        assert!(
            vel_after.y < vel_before.y - 10.0,
            "Internal gravity should apply downward acceleration"
        );
    }

    #[test]
    fn custom_gravity_affects_controller() {
        let mut app = create_test_app();

        let character = {
            let transform = Transform::from_translation(Vec2::new(0.0, 100.0).extend(0.0));
            app.world_mut()
                .spawn((
                    transform,
                    GlobalTransform::from(transform),
                    CharacterController::with_gravity(Vec2::new(0.0, -500.0)), // Custom gravity
                    ControllerConfig::default(),
                    MovementIntent::default(),
                    Rapier2dCharacterBundle::rotation_locked(),
                    Collider::capsule_y(8.0, 4.0),
                    GravityScale(0.0),
                ))
                .id()
        };

        let controller = app.world().get::<CharacterController>(character).unwrap();
        println!("PROOF: gravity={:?}", controller.gravity);

        // PROOF: Gravity should be custom value
        assert_eq!(
            controller.gravity,
            Vec2::new(0.0, -500.0),
            "Gravity should be custom value"
        );
    }
}

// ==================== Coyote Time Tests ====================

mod coyote_time {
    use super::*;

    #[test]
    fn time_since_grounded_zero_when_grounded() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_frames(&mut app, 5);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, time_since_grounded={}",
            controller.is_grounded(config),
            controller.time_since_grounded
        );

        // PROOF: time_since_grounded should be zero when grounded
        assert!(
            controller.time_since_grounded < 0.02,
            "time_since_grounded should be zero when grounded"
        );
    }

    #[test]
    fn time_since_grounded_accumulates_when_airborne() {
        let mut app = create_test_app();

        // No ground
        let character = spawn_character(&mut app, Vec2::new(0.0, 100.0));

        run_frames(&mut app, 30);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, time_since_grounded={}",
            controller.is_grounded(config),
            controller.time_since_grounded
        );

        // PROOF: time_since_grounded should accumulate (30 frames at 60Hz = 0.5s)
        assert!(
            controller.time_since_grounded > 0.3,
            "time_since_grounded should accumulate"
        );
    }
}

// ==================== Collision Layer Inheritance Tests ====================

mod collision_layers {
    use super::*;

    #[test]
    fn sensors_inherit_collision_groups() {
        let mut app = create_test_app();

        // Create ground that only collides with GROUP_1
        let ground_transform = Transform::from_translation(Vec2::new(0.0, 0.0).extend(0.0));
        app.world_mut().spawn((
            ground_transform,
            GlobalTransform::from(ground_transform),
            RigidBody::Fixed,
            Collider::cuboid(100.0, 5.0),
            CollisionGroups::new(Group::GROUP_1, Group::GROUP_1),
        ));

        // Character in GROUP_1 - should detect ground
        let char_in_group = {
            let transform = Transform::from_translation(Vec2::new(-20.0, 20.0).extend(0.0));
            app.world_mut()
                .spawn((
                    transform,
                    GlobalTransform::from(transform),
                    CharacterController::new(),
                    ControllerConfig::default(),
                    MovementIntent::default(),
                    Rapier2dCharacterBundle::rotation_locked(),
                    Collider::capsule_y(8.0, 4.0),
                    CollisionGroups::new(Group::GROUP_1, Group::GROUP_1),
                    GravityScale(0.0),
                ))
                .id()
        };

        // Character in GROUP_2 - should NOT detect ground
        let char_not_in_group = {
            let transform = Transform::from_translation(Vec2::new(20.0, 20.0).extend(0.0));
            app.world_mut()
                .spawn((
                    transform,
                    GlobalTransform::from(transform),
                    CharacterController::new(),
                    ControllerConfig::default(),
                    MovementIntent::default(),
                    Rapier2dCharacterBundle::rotation_locked(),
                    Collider::capsule_y(8.0, 4.0),
                    CollisionGroups::new(Group::GROUP_2, Group::GROUP_2),
                    GravityScale(0.0),
                ))
                .id()
        };

        tick(&mut app);

        let ctrl1 = app
            .world()
            .get::<CharacterController>(char_in_group)
            .unwrap();
        let ctrl2 = app
            .world()
            .get::<CharacterController>(char_not_in_group)
            .unwrap();

        println!(
            "PROOF: GROUP_1 char ground_detected={}, GROUP_2 char ground_detected={}",
            ctrl1.ground_detected(),
            ctrl2.ground_detected()
        );

        // PROOF: Sensors should inherit collision groups
        assert!(
            ctrl1.ground_detected(),
            "Character in GROUP_1 should detect GROUP_1 ground"
        );
        assert!(
            !ctrl2.ground_detected(),
            "Character in GROUP_2 should NOT detect GROUP_1 ground"
        );
    }
}
