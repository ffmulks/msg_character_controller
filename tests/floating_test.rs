use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;

/// Test that the floating spring system actually lifts the character off the ground
#[test]
fn character_floats_above_ground() {
    let mut app = App::new();

    // Add minimal plugins for testing
    app.add_plugins(MinimalPlugins);
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0));
    app.add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default());

    // Gravity is handled per-entity now, not as a resource

    // Spawn ground at Y = 0
    let ground = app
        .world_mut()
        .spawn((
            Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
            GlobalTransform::default(),
            RigidBody::Fixed,
            Collider::cuboid(100.0, 10.0), // Ground at Y=0, extends from -10 to +10
        ))
        .id();

    // Spawn character above ground
    let character = app
        .world_mut()
        .spawn((
            CharacterController::walking(),
            ControllerConfig::player()
                .with_float_height(20.0) // Should float 20 units above ground
                .with_spring(50000.0, 1000.0), // Very strong spring with damping
            CharacterGravity(Vec2::new(0.0, -980.0)), // REQUIRED for spring to know gravity!
            CharacterOrientation::default(),
            GroundInfo::default(),
            WalkIntent::default(),
            JumpRequest::default(),
            Transform::from_translation(Vec3::new(0.0, 100.0, 0.0)), // Start 100 units up
            GlobalTransform::default(),
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            Collider::capsule_y(5.0, 5.0), // Total height = 5 + 5 + 5 + 5 = 20 units
            GravityScale(0.0),             // Manual gravity
        ))
        .id();

    // Add gravity system
    app.add_systems(FixedUpdate, apply_test_gravity);

    // Add Time<Fixed> resource required for FixedUpdate
    app.insert_resource(Time::<Fixed>::from_seconds(1.0 / 60.0));

    // CRITICAL: The floating spring ONLY works if ground is detected!
    // We must run the full system chain for it to work
    println!("Testing with full character controller systems...");

    // Run physics for 100 steps (should be enough to settle)
    for i in 0..100 {
        // Run fixed update for physics
        app.update();

        // Check character position after settling
        if i > 50 {
            // Give it time to settle
            let character_transform = app.world().get::<Transform>(character).unwrap();
            let character_y = character_transform.translation.y;
            let velocity = app.world().get::<Velocity>(character).unwrap();
            let velocity_y = velocity.linvel.y;

            println!(
                "Step {}: Character Y = {:.2}, Velocity Y = {:.2}",
                i, character_y, velocity_y
            );

            // After settling, character should be floating
            if i > 80 && velocity_y.abs() < 50.0 {
                // Velocity should be near zero when settled
                // Ground top is at Y = 10 (collider extends from -10 to +10)
                // Character bottom is at center - 10 (capsule half-height)
                // So character center should be at: ground_top(10) + float_height(20) + capsule_half(10) = 40

                let expected_y = 40.0;
                let tolerance = 5.0;

                assert!(
                    (character_y - expected_y).abs() < tolerance,
                    "Character should float at Y={} (±{}), but is at Y={}",
                    expected_y,
                    tolerance,
                    character_y
                );

                println!("✓ Character is floating correctly at Y={:.2}", character_y);
                return; // Test passed
            }
        }
    }

    // If we get here, character didn't settle to floating position
    let character_transform = app.world().get::<Transform>(character).unwrap();
    let character_y = character_transform.translation.y;
    panic!(
        "Character did not settle to floating position! Final Y: {}",
        character_y
    );
}

/// Test that spring force is actually being applied
#[test]
fn spring_force_is_applied() {
    let mut app = App::new();

    app.add_plugins(MinimalPlugins);
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0));
    app.add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default());

    // Spawn ground
    app.world_mut().spawn((
        Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::cuboid(100.0, 10.0),
    ));

    // Spawn character with tracked external force
    let character = app
        .world_mut()
        .spawn((
            CharacterController::walking(),
            ControllerConfig::player()
                .with_float_height(20.0)
                .with_spring_strength(10000.0),
            CharacterGravity(Vec2::new(0.0, -980.0)), // REQUIRED!
            CharacterOrientation::default(),
            GroundInfo {
                detected: true,
                distance: 15.0, // Below float height
                normal: Vec2::Y,
                contact_point: Vec2::new(0.0, 10.0),
                slope_angle: 0.0,
                is_walkable: true,
                ground_entity: None,
                step_detected: false,
                step_height: 0.0,
                time_since_grounded: 0.0,
            },
            Transform::from_translation(Vec3::new(0.0, 25.0, 0.0)),
            GlobalTransform::default(),
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            Collider::capsule_y(5.0, 5.0),
            GravityScale(0.0),
        ))
        .id();

    // Add the character controller systems
    use msg_character_controller::systems;

    // Manually run the spring system since the test doesn't schedule it
    systems::apply_floating_spring::<Rapier2dBackend>(&mut app.world_mut());

    // Check that spring force was applied
    let force = app.world().get::<ExternalForce>(character).unwrap();

    println!("Applied force: {:?}", force.force);

    // Spring should apply upward force when below float height
    assert!(
        force.force.y > 0.0,
        "Spring should apply upward force when below float height, but force.y = {}",
        force.force.y
    );

    // Force should be significant (height_error = 20 - 15 = 5, force = 5 * 10000 = 50000)
    assert!(
        force.force.y > 40000.0,
        "Spring force should be strong (>40000), but is only {}",
        force.force.y
    );

    println!("✓ Spring force is being applied: {:.2} N", force.force.y);
}

fn apply_test_gravity(
    time: Res<Time<Fixed>>,
    mut query: Query<(&CharacterGravity, &mut Velocity)>,
) {
    let dt = time.delta_secs();
    for (gravity, mut velocity) in &mut query {
        println!("[GRAVITY] Applying {} with dt={}", gravity.0, dt);
        velocity.linvel += gravity.0 * dt;
    }
}
