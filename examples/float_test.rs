//! Float Test Example
//!
//! This example specifically tests if the character controller's floating spring
//! system is working. It spawns a character high in the air above a platform.
//!
//! If floating works: The character should descend and hover at float_height above the platform
//! If floating doesn't work: The character will fall through or sit directly on the platform
//!
//! Controls:
//! - **WASD/Arrow Keys**: Move
//! - **Space**: Jump (to test if we can jump from floating position)

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;

const PLAYER_RADIUS: f32 = 6.0;
const PLAYER_HALF_HEIGHT: f32 = 8.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Float Test - Character Controller".into(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        // Physics
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        // Character controller
        .add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default())
        // Resources
        .insert_resource(Gravity(Vec2::new(0.0, -980.0)))
        // Systems
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                handle_input,
                apply_gravity,
                debug_floating,
                camera_follow,
            ),
        )
        .run();
}

#[derive(Resource)]
struct Gravity(Vec2);

#[derive(Component)]
struct Player;

#[derive(Component)]
struct AffectedByGravity;

#[derive(Component)]
struct DebugText;

fn setup(mut commands: Commands) {
    // Camera
    commands.spawn((Camera2d, Transform::from_xyz(0.0, 0.0, 1000.0)));

    // Debug text
    commands.spawn((
        Text::new("FLOAT TEST\nSpawning character 200 units above platform...\n"),
        TextFont {
            font_size: 20.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        DebugText,
    ));

    // Ground platform (large so we can see floating clearly)
    commands.spawn((
        Transform::from_translation(Vec3::new(0.0, -200.0, 0.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::cuboid(400.0, 20.0),
        Sprite {
            color: Color::srgb(0.3, 0.3, 0.3),
            custom_size: Some(Vec2::new(800.0, 40.0)),
            ..default()
        },
    ));

    // Spawn player HIGH ABOVE the platform to test floating
    let spawn_pos = Vec2::new(0.0, 200.0); // 400 units above platform!

    // Calculate expected float position
    let float_height = 15.0; // From config
    let expected_hover_y = -200.0 + 20.0 + float_height; // Platform top + float height

    println!("=== FLOAT TEST SETUP ===");
    println!("Spawning player at Y: {}", spawn_pos.y);
    println!("Platform top at Y: {}", -180.0);
    println!("Expected float Y: {}", expected_hover_y);
    println!("Float height config: {}", float_height);

    commands
        .spawn((
            Player,
            AffectedByGravity,
            Transform::from_translation(spawn_pos.extend(1.0)),
            GlobalTransform::default(),
            Sprite {
                color: Color::srgb(0.2, 0.6, 0.9),
                custom_size: Some(Vec2::new(PLAYER_RADIUS * 2.0, PLAYER_HALF_HEIGHT * 2.0)),
                ..default()
            },
        ))
        .insert((
            // Character controller with explicit float height and gravity
            CharacterController::with_gravity(Vec2::new(0.0, -980.0)),
            ControllerConfig::player()
                .with_float_height(15.0) // Should float 15 units above ground
                .with_spring(20000.0, 500.0) // VERY strong spring
                .with_ground_cast_width(PLAYER_RADIUS),
            WalkIntent::default(),
            JumpRequest::default(),
        ))
        .insert((
            // Physics
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            LockedAxes::ROTATION_LOCKED,
            Collider::capsule_y(PLAYER_HALF_HEIGHT / 2.0, PLAYER_RADIUS),
            GravityScale(0.0), // Manual gravity
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
        ));
}

fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut query: Query<(&mut WalkIntent, &mut JumpRequest), With<Player>>,
) {
    for (mut walk_intent, mut jump_request) in &mut query {
        // Horizontal movement
        let mut direction = 0.0;
        if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
            direction -= 1.0;
        }
        if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
            direction += 1.0;
        }
        walk_intent.set(direction);

        // Jump
        if keyboard.just_pressed(KeyCode::Space) {
            jump_request.request(time.elapsed_secs());
        }
    }
}

fn apply_gravity(
    gravity: Res<Gravity>,
    time: Res<Time>,
    mut query: Query<(&mut Velocity, Option<&Grounded>), With<AffectedByGravity>>,
) {
    let dt = time.delta_secs();

    for (mut velocity, grounded) in &mut query {
        // Only apply gravity when not grounded
        if grounded.is_none() {
            velocity.linvel += gravity.0 * dt;
        }
    }
}

fn debug_floating(
    mut text_query: Query<(&mut Text, &mut TextColor), With<DebugText>>,
    player_query: Query<(&Transform, &Velocity, &CharacterController, Option<&Grounded>), With<Player>>,
) {
    let Ok((transform, velocity, controller, grounded)) = player_query.single() else {
        return;
    };

    let Ok((mut text, mut color)) = text_query.single_mut() else {
        return;
    };

    let grounded_str = if grounded.is_some() { "YES" } else { "NO" };
    let platform_top_y = -180.0;
    let distance_from_platform = transform.translation.y - platform_top_y;

    **text = format!(
        "FLOAT TEST\n\
         Player Y: {:.1}\n\
         Velocity Y: {:.1}\n\
         Distance from platform: {:.1}\n\
         Ground detected: {}\n\
         Ground distance: {:.1}\n\
         Grounded: {}\n\
         \n\
         EXPECTED: Should hover at ~15 units above platform\n\
         ACTUAL: {:.1} units above platform",
        transform.translation.y,
        velocity.linvel.y,
        distance_from_platform,
        controller.ground_detected(),
        controller.ground_distance(),
        grounded_str,
        distance_from_platform
    );

    // Color based on floating state
    if distance_from_platform > 10.0 && distance_from_platform < 20.0 && velocity.linvel.y.abs() < 50.0 {
        color.0 = Color::srgb(0.2, 0.8, 0.2); // Green
    } else if controller.ground_detected() {
        color.0 = Color::srgb(0.9, 0.9, 0.2); // Yellow
    } else {
        color.0 = Color::srgb(0.9, 0.3, 0.3); // Red
    }
}

fn camera_follow(
    player_query: Query<&Transform, (With<Player>, Without<Camera2d>)>,
    mut camera_query: Query<&mut Transform, With<Camera2d>>,
) {
    let Ok(player_transform) = player_query.single() else {
        return;
    };

    let Ok(mut camera_transform) = camera_query.single_mut() else {
        return;
    };

    camera_transform.translation.x = player_transform.translation.x;
    camera_transform.translation.y = player_transform.translation.y;
}
