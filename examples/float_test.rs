//! Float Test Example
//!
//! This example specifically tests if the character controller's floating spring
//! system is working. It spawns a character high in the air above a platform.
//!
//! If floating works: The character should descend and hover at float_height above the platform
//! If floating doesn't work: The character will fall through or sit directly on the platform
//!
//! ## Controls
//! - **A/D** or **Left/Right**: Move horizontally
//! - **W/Up**: Jump
//! - **Space** (hold): Propulsion (fly upward)
//! - **S/Down** (hold): Propulsion (fly downward)

mod helpers;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use bevy_rapier2d::prelude::*;
use helpers::{
    CharacterControllerUiPlugin, ControlsPlugin, DefaultControllerSettings, Player, SpawnConfig,
};
use msg_character_controller::prelude::*;

const PLAYER_RADIUS: f32 = 6.0;
const PLAYER_HALF_HEIGHT: f32 = 8.0;

fn spawn_position() -> Vec2 {
    Vec2::new(0.0, 200.0) // 400 units above platform!
}

fn default_config() -> ControllerConfig {
    ControllerConfig::player()
        .with_float_height(2.0) // Gap between capsule bottom and ground
        .with_spring(20000.0, 500.0) // VERY strong spring
        .with_ground_cast_width(PLAYER_RADIUS)
}

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
        // Controls (input handling and camera follow)
        .add_plugins(ControlsPlugin::default())
        // Egui for settings UI
        .add_plugins(EguiPlugin::default())
        // Configure spawn position and default settings for the UI plugin
        .insert_resource(SpawnConfig::new(spawn_position()))
        .insert_resource(DefaultControllerSettings::new(
            default_config(),
            Vec2::new(0.0, -980.0),
        ))
        // Character controller UI panels (unified plugin with settings + diagnostics)
        .add_plugins(CharacterControllerUiPlugin::<Player>::default())
        // Systems
        .add_systems(Startup, setup)
        .add_systems(Update, debug_floating)
        .run();
}

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
        Pickable::IGNORE, // Prevent this UI element from blocking mouse clicks
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
    let spawn_pos = spawn_position();

    // Calculate expected float position
    // float_height is the gap between the BOTTOM of the capsule and the ground
    let float_height = 2.0; // 2 pixels gap between capsule bottom and ground
    let collider_bottom_offset = PLAYER_HALF_HEIGHT / 2.0 + PLAYER_RADIUS; // 4 + 6 = 10
    let platform_top = -200.0 + 20.0; // Platform at y=-200, half-height=20, so top at -180
    let expected_hover_y = platform_top + float_height + collider_bottom_offset; // -180 + 2 + 10 = -168

    println!("=== FLOAT TEST SETUP ===");
    println!("Spawning player at Y: {}", spawn_pos.y);
    println!("Platform top at Y: {}", platform_top);
    println!(
        "Float height (gap from capsule bottom to ground): {}",
        float_height
    );
    println!("Collider bottom offset: {}", collider_bottom_offset);
    println!("Expected center Y: {}", expected_hover_y);

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
            // Character controller with explicit float height and gravity
            CharacterController::with_gravity(Vec2::new(0.0, -980.0)),
            default_config(),
            MovementIntent::default(),
        ))
        .insert((
            // Physics
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            LockedAxes::ROTATION_LOCKED,
            Collider::capsule_y(PLAYER_HALF_HEIGHT / 2.0, PLAYER_RADIUS),
            GravityScale(0.0), // Gravity is applied internally by the controller
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
        ));
}

fn debug_floating(
    mut text_query: Query<(&mut Text, &mut TextColor), With<DebugText>>,
    player_query: Query<
        (
            &Transform,
            &Velocity,
            &CharacterController,
            &ControllerConfig,
        ),
        With<Player>,
    >,
) {
    let Ok((transform, velocity, controller, config)) = player_query.single() else {
        return;
    };

    let Ok((mut text, mut color)) = text_query.single_mut() else {
        return;
    };

    let grounded_str = if controller.is_grounded(config) {
        "YES"
    } else {
        "NO"
    };
    let platform_top_y = -180.0;
    let collider_bottom_offset = PLAYER_HALF_HEIGHT / 2.0 + PLAYER_RADIUS; // 10
    let capsule_bottom_y = transform.translation.y - collider_bottom_offset;
    let gap_from_ground = capsule_bottom_y - platform_top_y;

    **text = format!(
        "FLOAT TEST\n\
         Player center Y: {:.1}\n\
         Capsule bottom Y: {:.1}\n\
         Gap from ground: {:.1}\n\
         Velocity Y: {:.1}\n\
         Ground detected: {}\n\
         Ground distance (from center): {:.1}\n\
         Grounded: {}\n\
         \n\
         EXPECTED: Capsule bottom should hover ~2 px above platform\n\
         ACTUAL: {:.1} px gap",
        transform.translation.y,
        capsule_bottom_y,
        gap_from_ground,
        velocity.linvel.y,
        controller.ground_detected(),
        controller.ground_distance(),
        grounded_str,
        gap_from_ground
    );

    // Color based on floating state (gap should be around 2 pixels)
    if gap_from_ground > 0.0 && gap_from_ground < 5.0 && velocity.linvel.y.abs() < 50.0 {
        color.0 = Color::srgb(0.2, 0.8, 0.2); // Green - stable floating
    } else if controller.ground_detected() {
        color.0 = Color::srgb(0.9, 0.9, 0.2); // Yellow - ground detected but not stable
    } else {
        color.0 = Color::srgb(0.9, 0.3, 0.3); // Red - no ground
    }
}
