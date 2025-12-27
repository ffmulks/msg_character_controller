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
const FLOAT_HEIGHT: f32 = 15.0;

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
        // Physics - disable Rapier's global gravity since we use internal gravity
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        // Character controller with INTERNAL gravity (default)
        // This means the plugin handles gravity - DO NOT apply gravity externally!
        .add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default())
        // Systems
        .add_systems(Startup, setup)
        .add_systems(Update, (handle_input, debug_floating, camera_follow))
        .run();
}

#[derive(Component)]
struct Player;

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
    let expected_hover_y = -200.0 + 20.0 + FLOAT_HEIGHT; // Platform top + float height

    println!("=== FLOAT TEST SETUP ===");
    println!("Spawning player at Y: {}", spawn_pos.y);
    println!("Platform top at Y: {}", -180.0);
    println!("Expected float Y: {}", expected_hover_y);
    println!("Float height config: {}", FLOAT_HEIGHT);

    commands
        .spawn((
            Player,
            Transform::from_translation(spawn_pos.extend(1.0)),
            GlobalTransform::default(),
            Sprite {
                color: Color::srgb(0.2, 0.6, 0.9),
                custom_size: Some(Vec2::new(PLAYER_RADIUS * 2.0, PLAYER_HALF_HEIGHT * 2.0)),
                ..default()
            },
        ))
        .insert((
            // Character controller with explicit gravity
            // The plugin uses INTERNAL gravity mode by default, so it applies gravity for us
            CharacterController::with_gravity(Vec2::new(0.0, -980.0)),
            ControllerConfig::player()
                .with_float_height(FLOAT_HEIGHT)
                .with_spring(15000.0, 400.0) // Strong spring with good damping
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
            // IMPORTANT: Set GravityScale to 0 because we use internal gravity
            // The plugin applies gravity as a force, not through Rapier's gravity system
            GravityScale(0.0),
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

fn debug_floating(
    mut text_query: Query<(&mut Text, &mut TextColor), With<DebugText>>,
    player_query: Query<
        (&Transform, &Velocity, &CharacterController, Option<&Grounded>),
        With<Player>,
    >,
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
         EXPECTED: Should hover at ~{} units above platform\n\
         ACTUAL: {:.1} units above platform",
        transform.translation.y,
        velocity.linvel.y,
        distance_from_platform,
        controller.ground_detected(),
        controller.ground_distance(),
        grounded_str,
        FLOAT_HEIGHT,
        distance_from_platform
    );

    // Color based on floating state
    if distance_from_platform > (FLOAT_HEIGHT - 3.0)
        && distance_from_platform < (FLOAT_HEIGHT + 3.0)
        && velocity.linvel.y.abs() < 50.0
    {
        color.0 = Color::srgb(0.2, 0.8, 0.2); // Green - floating correctly
    } else if controller.ground_detected() {
        color.0 = Color::srgb(0.9, 0.9, 0.2); // Yellow - ground detected but not at target
    } else {
        color.0 = Color::srgb(0.9, 0.3, 0.3); // Red - no ground detected
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
