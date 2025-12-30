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

mod helpers;

use bevy::prelude::*;
use bevy::sprite::ColorMaterial;
use bevy_egui::EguiPlugin;
use bevy_rapier2d::prelude::*;
use helpers::{
    CharacterControllerUiPlugin, ControlsPlugin, DefaultControllerSettings, Player, SpawnConfig,
    create_capsule_mesh, create_circle_mesh, create_rectangle_mesh, create_triangle_mesh,
};
use msg_character_controller::prelude::*;

// ==================== Constants ====================

const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

const BOX_WIDTH: f32 = 800.0;
const BOX_HEIGHT: f32 = 600.0;
const WALL_THICKNESS: f32 = 20.0;

const PLATFORM_WIDTH: f32 = 200.0;
const PLATFORM_HEIGHT: f32 = 20.0;
const PLATFORM_Y: f32 = 100.0;

const PX_PER_M: f32 = 10.0; // Pixels per meter for Rapier

// ==================== Main ====================

fn spawn_position() -> Vec2 {
    Vec2::new(-200.0, -BOX_HEIGHT / 2.0 + WALL_THICKNESS + 50.0)
}

fn default_gravity() -> Vec2 {
    Vec2::new(0.0, -9.81 * PX_PER_M)
}

fn default_config() -> ControllerConfig {
    ControllerConfig::player().with_float_height(6.0)
}

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
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(
            PX_PER_M,
        ))
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
            default_gravity(),
        ))
        // Character controller UI panels (unified plugin with settings + diagnostics)
        .add_plugins(CharacterControllerUiPlugin::<Player>::default())
        // Systems
        .add_systems(Startup, setup)
        .run();
}

// ==================== Setup ====================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Spawn environment
    spawn_box(&mut commands, &mut meshes, &mut materials);
    spawn_obstacles(&mut commands, &mut meshes, &mut materials);
    spawn_slope(&mut commands, &mut meshes, &mut materials);

    // Spawn player
    spawn_player(&mut commands, &mut meshes, &mut materials);

    // UI instructions - use Pickable::IGNORE to prevent blocking mouse events
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
        Pickable::IGNORE, // Prevent this UI element from blocking mouse clicks
    ));
}

fn spawn_box(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    let half_width = BOX_WIDTH / 2.0;
    let half_height = BOX_HEIGHT / 2.0;
    let half_wall = WALL_THICKNESS / 2.0;

    // Floor
    spawn_static_collider(
        commands,
        meshes,
        materials,
        Vec2::new(0.0, -half_height - half_wall),
        Vec2::new(half_width, half_wall),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Ceiling
    spawn_static_collider(
        commands,
        meshes,
        materials,
        Vec2::new(0.0, half_height + half_wall),
        Vec2::new(half_width, half_wall),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Left wall
    spawn_static_collider(
        commands,
        meshes,
        materials,
        Vec2::new(-half_width + half_wall, 0.0),
        Vec2::new(half_wall, half_height),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Right wall
    spawn_static_collider(
        commands,
        meshes,
        materials,
        Vec2::new(half_width - half_wall, 0.0),
        Vec2::new(half_wall, half_height),
        Color::srgb(0.3, 0.3, 0.3),
    );
}

fn spawn_obstacles(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    spawn_static_collider(
        commands,
        meshes,
        materials,
        Vec2::new(0.0, PLATFORM_Y),
        Vec2::new(PLATFORM_WIDTH / 2.0, PLATFORM_HEIGHT / 2.0),
        Color::srgb(0.4, 0.5, 0.3),
    );

    for i in 0..=5 {
        spawn_ball_static_collider(
            commands,
            meshes,
            materials,
            Vec2::new(50.0 + 13.0 * i as f32, -BOX_HEIGHT / 2.0 + 10.0 * i as f32),
            5.0,
            Color::srgb(0.8, 0.2, 0.2),
        );
    }

    for i in 0..=5 {
        spawn_ball_static_collider(
            commands,
            meshes,
            materials,
            Vec2::new(
                -BOX_WIDTH / 2.0 + WALL_THICKNESS + 150.0 + 20.0 * i as f32,
                -BOX_HEIGHT / 2.0 + 5.0,
            ),
            5.0,
            Color::srgb(0.8, 0.2, 0.2),
        );
    }

    for i in 0..=5 {
        spawn_ball_static_collider(
            commands,
            meshes,
            materials,
            Vec2::new(
                -BOX_WIDTH / 2.0 + WALL_THICKNESS + 20.0 + 20.0 * i as f32,
                -BOX_HEIGHT / 2.0,
            ),
            5.0,
            Color::srgb(0.8, 0.2, 0.2),
        );
    }
}

fn spawn_slope(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    // Create a triangle slope using a convex hull collider
    // Position it on the right side of the box
    let slope_x = 250.0;
    let slope_y = -BOX_HEIGHT / 2.0;

    // Triangle vertices (relative to center)
    let vertices = vec![
        Vec2::new(-80.0, 0.0),  // Bottom left
        Vec2::new(80.0, 0.0),   // Bottom right
        Vec2::new(80.0, 100.0), // Top right
    ];

    // Use convex_hull which works well for triangles
    let collider = Collider::convex_hull(&vertices).expect("Failed to create slope collider");

    // Create a triangle mesh that matches the collider
    let triangle_vertices = [vertices[0], vertices[1], vertices[2]];
    let mesh = meshes.add(create_triangle_mesh(&triangle_vertices));
    let material = materials.add(ColorMaterial::from_color(Color::srgb(0.5, 0.4, 0.3)));

    commands.spawn((
        Transform::from_translation(Vec3::new(slope_x, slope_y, 0.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        collider,
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

fn spawn_static_collider(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    half_size: Vec2,
    color: Color,
) {
    let mesh = meshes.add(create_rectangle_mesh(half_size.x, half_size.y));
    let material = materials.add(ColorMaterial::from_color(color));

    commands.spawn((
        Transform::from_translation(position.extend(0.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::cuboid(half_size.x, half_size.y),
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

fn spawn_ball_static_collider(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    radius: f32,
    color: Color,
) {
    let mesh = meshes.add(create_circle_mesh(radius, 24));
    let material = materials.add(ColorMaterial::from_color(color));

    commands.spawn((
        Transform::from_translation(position.extend(0.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::ball(radius),
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

fn spawn_player(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    let spawn_pos = spawn_position();

    // Create capsule mesh matching the collider
    let mesh = meshes.add(create_capsule_mesh(PLAYER_HALF_HEIGHT, PLAYER_RADIUS, 12));
    let material = materials.add(ColorMaterial::from_color(Color::srgb(0.2, 0.6, 0.9)));

    commands
        .spawn((
            Player,
            Transform::from_translation(spawn_pos.extend(1.0)),
            GlobalTransform::default(),
            Mesh2d(mesh),
            MeshMaterial2d(material),
        ))
        .insert((
            // Character controller with gravity
            CharacterController::with_gravity(default_gravity()),
            default_config(),
            MovementIntent::default(),
        ))
        .insert((
            // Physics
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            Collider::capsule_y(PLAYER_HALF_HEIGHT, PLAYER_RADIUS),
            GravityScale(0.0), // Gravity is applied internally by the controller
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
        ));
}
