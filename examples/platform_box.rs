//! Platform Box Example
//!
//! A playable example with a character in a box environment featuring:
//! - A floor
//! - Walls on both sides
//! - A platform in the middle
//! - A triangle slope on the right
//!
//! ## Controls
//! - **A/D** or **Left/Right**: Move horizontally
//! - **W/Up**: Jump
//! - **Space** (hold): Propulsion (fly upward)
//! - **S/Down** (hold): Propulsion (fly downward)
//!
//! The camera follows the player.

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;

// ==================== Constants ====================

const PLAYER_SIZE: f32 = 16.0;
const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

const BOX_WIDTH: f32 = 800.0;
const BOX_HEIGHT: f32 = 600.0;
const WALL_THICKNESS: f32 = 20.0;

const PLATFORM_WIDTH: f32 = 200.0;
const PLATFORM_HEIGHT: f32 = 20.0;
const PLATFORM_Y: f32 = 100.0;

// ==================== Components ====================

/// Marker for the player entity.
#[derive(Component)]
struct Player;

/// Marker for entities that need gravity applied.
#[derive(Component)]
struct AffectedByGravity;

// ==================== Main ====================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Platform Box - Character Controller Example".into(),
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
        // Systems
        .add_systems(Startup, setup)
        .add_systems(Update, (handle_input, apply_gravity, camera_follow))
        .run();
}

// ==================== Setup ====================

fn setup(mut commands: Commands) {
    // Camera
    commands.spawn((Camera2d, Transform::from_xyz(0.0, 0.0, 1000.0)));

    // Spawn environment
    spawn_box(&mut commands);
    spawn_platform(&mut commands);
    spawn_slope(&mut commands);

    // Spawn player
    spawn_player(&mut commands);

    // UI instructions
    commands.spawn((
        Text::new("A/D: Move | W: Jump | Space: Propel Up | S: Propel Down"),
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
    ));
}

fn spawn_box(commands: &mut Commands) {
    let half_width = BOX_WIDTH / 2.0;
    let half_height = BOX_HEIGHT / 2.0;
    let half_wall = WALL_THICKNESS / 2.0;

    // Floor
    spawn_static_collider(
        commands,
        Vec2::new(0.0, -half_height),
        Vec2::new(half_width, half_wall),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Ceiling
    spawn_static_collider(
        commands,
        Vec2::new(0.0, half_height),
        Vec2::new(half_width, half_wall),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Left wall
    spawn_static_collider(
        commands,
        Vec2::new(-half_width, 0.0),
        Vec2::new(half_wall, half_height),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Right wall
    spawn_static_collider(
        commands,
        Vec2::new(half_width, 0.0),
        Vec2::new(half_wall, half_height),
        Color::srgb(0.3, 0.3, 0.3),
    );
}

fn spawn_platform(commands: &mut Commands) {
    spawn_static_collider(
        commands,
        Vec2::new(0.0, PLATFORM_Y),
        Vec2::new(PLATFORM_WIDTH / 2.0, PLATFORM_HEIGHT / 2.0),
        Color::srgb(0.4, 0.5, 0.3),
    );
}

fn spawn_slope(commands: &mut Commands) {
    // Create a triangle slope using a convex hull collider
    // Position it on the right side of the box
    let slope_x = 250.0;
    let slope_y = -BOX_HEIGHT / 2.0 + WALL_THICKNESS / 2.0;

    // Triangle vertices (relative to center)
    let vertices = vec![
        Vec2::new(-80.0, 0.0),  // Bottom left
        Vec2::new(80.0, 0.0),   // Bottom right
        Vec2::new(80.0, 100.0), // Top right
    ];

    // Use convex_hull which works well for triangles
    let collider = Collider::convex_hull(&vertices).expect("Failed to create slope collider");

    commands.spawn((
        Transform::from_translation(Vec3::new(slope_x, slope_y, 0.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        collider,
        Sprite {
            color: Color::srgb(0.5, 0.4, 0.3),
            custom_size: Some(Vec2::new(160.0, 100.0)),
            ..default()
        },
    ));
}

fn spawn_static_collider(commands: &mut Commands, position: Vec2, half_size: Vec2, color: Color) {
    commands.spawn((
        Transform::from_translation(position.extend(0.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::cuboid(half_size.x, half_size.y),
        Sprite {
            color,
            custom_size: Some(half_size * 2.0),
            ..default()
        },
    ));
}

fn spawn_player(commands: &mut Commands) {
    let spawn_pos = Vec2::new(-200.0, -BOX_HEIGHT / 2.0 + WALL_THICKNESS + 50.0);

    commands
        .spawn((
            Player,
            AffectedByGravity,
            Transform::from_translation(spawn_pos.extend(1.0)),
            GlobalTransform::default(),
            Sprite {
                color: Color::srgb(0.2, 0.6, 0.9),
                custom_size: Some(Vec2::new(PLAYER_RADIUS * 2.0, PLAYER_SIZE)),
                ..default()
            },
        ))
        .insert((
            // Character controller with gravity
            CharacterController::with_gravity(Vec2::new(0.0, -980.0)),
            ControllerConfig::player()
                // Capsule total half-height = half_length + radius = 4 + 6 = 10
                // We want to float 5 units above ground, so total = 10 + 5 = 15
                .with_float_height(15.0)
                .with_ground_cast_width(PLAYER_RADIUS),
            WalkIntent::default(),
            PropulsionIntent::default(),
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
            GravityScale(0.0), // We apply gravity manually
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
        ));
}

// ==================== Input Handling ====================

fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut query: Query<(&mut WalkIntent, &mut PropulsionIntent, &mut JumpRequest), With<Player>>,
) {
    for (mut walk_intent, mut propulsion, mut jump_request) in &mut query {
        // Horizontal input (A/D or Left/Right)
        let mut horizontal = 0.0;
        if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
            horizontal -= 1.0;
        }
        if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
            horizontal += 1.0;
        }

        walk_intent.set(horizontal);

        // Vertical propulsion (Space = up, S/Down = down)
        let mut vertical = 0.0;
        if keyboard.pressed(KeyCode::Space) {
            vertical += 1.0;
        }
        if keyboard.pressed(KeyCode::KeyS) || keyboard.pressed(KeyCode::ArrowDown) {
            vertical -= 1.0;
        }
        propulsion.set(vertical);

        // Jump on W or Up (just pressed)
        if keyboard.just_pressed(KeyCode::KeyW) || keyboard.just_pressed(KeyCode::ArrowUp) {
            jump_request.request(time.elapsed_secs());
        }
    }
}

// ==================== Gravity System ====================

/// Applies gravity to entities with the AffectedByGravity component.
/// This is separate from the character controller's floating spring system.
fn apply_gravity(
    time: Res<Time<Fixed>>,
    mut query: Query<
        (&CharacterController, &mut Velocity, Option<&Grounded>),
        With<AffectedByGravity>,
    >,
) {
    let dt = time.delta_secs();

    for (controller, mut velocity, grounded) in &mut query {
        // Only apply gravity when not grounded
        if grounded.is_none() {
            velocity.linvel += controller.gravity * dt;
        }
    }
}

// ==================== Camera ====================

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

    // Smooth camera follow
    let target = player_transform.translation.xy();
    let current = camera_transform.translation.xy();
    let smoothed = current.lerp(target, 0.1);
    camera_transform.translation.x = smoothed.x;
    camera_transform.translation.y = smoothed.y;
}
