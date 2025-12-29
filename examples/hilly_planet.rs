//! Hilly Planet Example
//!
//! A playable example with a character walking on a spherical planet with hilly terrain:
//! - Uses 16 pixels per meter units
//! - Spherical planet with procedural hilly surface
//! - Realistic gravity: 9.81 * 16 = 156.96 px/s²
//! - Propulsion system with gravity compensation for upward thrust
//!
//! ## Controls
//! - **A/D** or **Left/Right**: Move horizontally
//! - **W/Up**: Jump (single impulse when grounded)
//! - **Space** (hold): Propulsion (thrust upward with gravity compensation)
//! - **S/Down** (hold): Propulsion (thrust downward)
//!
//! The propulsion system provides vertical thrust that is automatically
//! boosted by gravity magnitude to help counteract it when going up.

mod helpers;

use bevy::prelude::*;
use bevy::sprite::ColorMaterial;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use bevy_rapier2d::prelude::*;
use helpers::{
    create_capsule_mesh, create_polygon_mesh, create_rectangle_mesh, create_triangle_mesh,
    CharacterControllerUiPlugin, CharacterControllerUiState, ControlsPlugin,
    DefaultControllerSettings, Player, SpawnConfig,
};
use msg_character_controller::prelude::*;
use std::f32::consts::{PI, TAU};

// ==================== Constants ====================

// Physics scale: 16 pixels = 1 meter
const PIXELS_PER_METER: f32 = 16.0;

// Gravity: 9.81 m/s² * 16 px/m = 156.96 px/s²
const GRAVITY_STRENGTH: f32 = 9.81 * PIXELS_PER_METER;

// Player dimensions (in pixels)
const PLAYER_SIZE: f32 = 16.0;
const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

// Planet configuration
const PLANET_CENTER: Vec2 = Vec2::ZERO;
const PLANET_BASE_RADIUS: f32 = 300.0;
const HILL_AMPLITUDE: f32 = 30.0; // Height variation for hills
const HILL_FREQUENCY: usize = 12; // Number of major hills around the planet
const PLANET_SEGMENTS: usize = 96; // Segments for planet surface

// ==================== Components ====================

/// Component storing the planet configuration.
#[derive(Resource)]
struct PlanetConfig {
    center: Vec2,
    gravity_strength: f32,
}

impl Default for PlanetConfig {
    fn default() -> Self {
        Self {
            center: PLANET_CENTER,
            gravity_strength: GRAVITY_STRENGTH,
        }
    }
}

// ==================== Config Functions ====================

/// Generate the radius at a given angle with hills.
fn planet_radius_at_angle(angle: f32) -> f32 {
    // Create multiple overlapping sine waves for interesting terrain
    let hill1 = (angle * HILL_FREQUENCY as f32).sin() * HILL_AMPLITUDE;
    let hill2 = (angle * (HILL_FREQUENCY as f32 * 2.3)).sin() * (HILL_AMPLITUDE * 0.4);
    let hill3 = (angle * (HILL_FREQUENCY as f32 * 0.5)).sin() * (HILL_AMPLITUDE * 0.6);

    PLANET_BASE_RADIUS + hill1 + hill2 + hill3
}

fn spawn_position() -> Vec2 {
    // Spawn on top of the planet
    let spawn_angle = PI / 2.0;
    let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
    let surface_radius = planet_radius_at_angle(spawn_angle);
    PLANET_CENTER + direction * (surface_radius + 40.0)
}

fn default_config() -> ControllerConfig {
    ControllerConfig::player()
        .with_float_height(PLAYER_HALF_HEIGHT)
        .with_ground_cast_width(PLAYER_RADIUS)
        .with_upright_torque_enabled(false) // We handle rotation via orientation
}

// ==================== Main ====================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Hilly Planet - 16px/m Physics".into(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        // Physics with 16 pixels per meter
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(
            PIXELS_PER_METER,
        ))
        .add_plugins(RapierDebugRenderPlugin::default())
        // Character controller
        .add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default())
        // Controls (input handling only - we have custom camera follow for planet)
        .add_plugins(ControlsPlugin::input_only())
        // Egui for settings UI
        .add_plugins(EguiPlugin::default())
        // Resources
        .init_resource::<PlanetConfig>()
        // Configure spawn position and default settings for the UI plugin
        .insert_resource(SpawnConfig::with_dynamic_position(spawn_position))
        .insert_resource(DefaultControllerSettings::new(
            default_config(),
            // Initial gravity will be updated by the orientation system
            -Vec2::Y * GRAVITY_STRENGTH,
        ))
        // Character controller UI panels (unified plugin with settings + diagnostics)
        .add_plugins(CharacterControllerUiPlugin::<Player>::default())
        // Systems
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            // Update orientation and gravity before controller systems
            update_player_orientation_and_gravity.before(
                msg_character_controller::systems::accumulate_spring_force::<Rapier2dBackend>,
            ),
        )
        .add_systems(Update, camera_follow)
        // Extra settings UI for planet-specific configuration
        .add_systems(EguiPrimaryContextPass, planet_gravity_settings_ui)
        .run();
}

// ==================== Setup ====================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera2d,
        Transform::from_xyz(0.0, PLANET_BASE_RADIUS + 100.0, 0.0),
    ));

    // Spawn hilly planet
    spawn_hilly_planet(&mut commands, &mut meshes, &mut materials);

    // Spawn player on top of planet
    spawn_player(&mut commands, &mut meshes, &mut materials);

    // UI instructions
    commands.spawn((
        Text::new("A/D: Move | W: Jump | Space: Propel Up | S: Propel Down"),
        TextFont {
            font_size: 18.0,
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

    // Physics info
    commands.spawn((
        Text::new(format!(
            "16 px/meter | Gravity: {:.2} px/s² (9.81 m/s²)",
            GRAVITY_STRENGTH
        )),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::srgb(0.7, 0.7, 0.7)),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        Pickable::IGNORE, // Prevent this UI element from blocking mouse clicks
    ));

    // Propulsion info
    commands.spawn((
        Text::new("Hold Space to propel upward (with gravity compensation)"),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::srgb(0.6, 0.6, 0.8)),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(30.0),
            left: Val::Px(10.0),
            ..default()
        },
        Pickable::IGNORE, // Prevent this UI element from blocking mouse clicks
    ));
}

fn spawn_hilly_planet(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    // Generate vertices for the hilly planet surface
    let mut vertices: Vec<Vec2> = Vec::with_capacity(PLANET_SEGMENTS);

    for i in 0..PLANET_SEGMENTS {
        let angle = (i as f32 / PLANET_SEGMENTS as f32) * TAU;
        let radius = planet_radius_at_angle(angle);
        let x = angle.cos() * radius;
        let y = angle.sin() * radius;
        vertices.push(Vec2::new(x, y));
    }

    // Create a polyline collider for the planet surface
    // We need to close the loop by connecting back to the first vertex
    let mut indices: Vec<[u32; 2]> = Vec::with_capacity(PLANET_SEGMENTS);
    for i in 0..PLANET_SEGMENTS {
        let next = (i + 1) % PLANET_SEGMENTS;
        indices.push([i as u32, next as u32]);
    }

    let collider = Collider::polyline(vertices.clone(), Some(indices));

    // Create a polygon mesh that matches the hilly surface
    let mesh = meshes.add(create_polygon_mesh(&vertices));
    let material = materials.add(ColorMaterial::from_color(Color::srgb(0.2, 0.35, 0.2)));

    commands.spawn((
        Transform::from_translation(PLANET_CENTER.extend(-1.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        collider,
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));

    // Add some distinct landmark features on the surface

    // A flat platform area (good landing spot)
    spawn_surface_platform(commands, meshes, materials, PI * 0.5, 60.0, 8.0); // Top

    // A ramp/slope structure
    spawn_surface_slope(commands, meshes, materials, PI * 0.25); // Upper right

    // Another platform on the side
    spawn_surface_platform(commands, meshes, materials, PI, 50.0, 6.0); // Left side

    // Small stepping stones
    spawn_surface_platform(commands, meshes, materials, PI * 1.5, 30.0, 5.0); // Bottom
    spawn_surface_platform(commands, meshes, materials, PI * 1.75, 25.0, 5.0); // Bottom right
}

/// Spawn a platform on the planet surface at a given angle.
fn spawn_surface_platform(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    angle: f32,
    width: f32,
    height: f32,
) {
    let direction = Vec2::new(angle.cos(), angle.sin());
    let surface_radius = planet_radius_at_angle(angle);
    let position = PLANET_CENTER + direction * (surface_radius + height / 2.0 + 5.0);

    // Rotation to align with surface (tangent)
    let rotation = Quat::from_rotation_z(angle - PI / 2.0);

    let mesh = meshes.add(create_rectangle_mesh(width / 2.0, height / 2.0));
    let material = materials.add(ColorMaterial::from_color(Color::srgb(0.4, 0.5, 0.35)));

    commands.spawn((
        Transform::from_translation(position.extend(0.0)).with_rotation(rotation),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::cuboid(width / 2.0, height / 2.0),
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

/// Spawn a triangular slope on the planet surface.
fn spawn_surface_slope(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    angle: f32,
) {
    let direction = Vec2::new(angle.cos(), angle.sin());
    let surface_radius = planet_radius_at_angle(angle);
    let position = PLANET_CENTER + direction * (surface_radius + 20.0);

    // Rotation to align with surface
    let rotation = Quat::from_rotation_z(angle - PI / 2.0);

    // Triangle vertices in local space
    let vertices = vec![
        Vec2::new(-30.0, 0.0), // Bottom left
        Vec2::new(30.0, 0.0),  // Bottom right
        Vec2::new(30.0, 40.0), // Top right
    ];

    let collider = Collider::convex_hull(&vertices).expect("Failed to create slope collider");

    // Create triangle mesh matching the collider
    let triangle_vertices = [vertices[0], vertices[1], vertices[2]];
    let mesh = meshes.add(create_triangle_mesh(&triangle_vertices));
    let material = materials.add(ColorMaterial::from_color(Color::srgb(0.5, 0.4, 0.3)));

    commands.spawn((
        Transform::from_translation(position.extend(0.0)).with_rotation(rotation),
        GlobalTransform::default(),
        RigidBody::Fixed,
        collider,
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

fn spawn_player(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    // Spawn on top of the planet
    let spawn_angle = PI / 2.0; // Top of planet
    let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
    let surface_radius = planet_radius_at_angle(spawn_angle);
    let spawn_pos = PLANET_CENTER + direction * (surface_radius + 40.0);

    // Initial gravity pointing toward planet center
    // Up direction is derived from gravity via controller.ideal_up()
    let initial_gravity = -direction * GRAVITY_STRENGTH;

    // Create capsule mesh matching the collider
    let mesh = meshes.add(create_capsule_mesh(PLAYER_HALF_HEIGHT / 2.0, PLAYER_RADIUS, 12));
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
            // Character controller with initial gravity pointing toward planet
            CharacterController::with_gravity(initial_gravity),
            default_config(),
            MovementIntent::default(),
        ))
        .insert((
            // Physics
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            Collider::capsule_y(PLAYER_HALF_HEIGHT / 2.0, PLAYER_RADIUS),
            GravityScale(0.0), // Gravity is applied internally by the controller
            Damping {
                linear_damping: 0.0,
                angular_damping: 5.0, // Some angular damping for stability
            },
        ));
}

// ==================== Planetary Systems ====================

/// Updates the player's gravity to match the planet.
///
/// Gravity points toward the planet center and is stored in CharacterController.
/// The up direction is derived from gravity via controller.ideal_up().
/// The internal gravity system then applies it when not grounded.
fn update_player_orientation_and_gravity(
    planet: Res<PlanetConfig>,
    mut query: Query<(&mut Transform, &mut CharacterController), With<Player>>,
) {
    for (mut transform, mut controller) in &mut query {
        let position = transform.translation.xy();
        let to_player = position - planet.center;
        let new_up = to_player.normalize_or_zero();

        if new_up != Vec2::ZERO {
            // Update gravity to point toward planet center
            // Up direction is automatically derived from gravity via controller.ideal_up()
            controller.gravity = -new_up * planet.gravity_strength;

            // Rotate the transform to visually match the up direction
            let angle = new_up.to_angle() - PI / 2.0;
            transform.rotation = Quat::from_rotation_z(angle);
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

    // Smooth camera follow - position behind player relative to planet
    let player_pos = player_transform.translation.xy();
    let to_player = player_pos - PLANET_CENTER;
    let camera_distance = to_player.length() + 150.0;

    let camera_target = PLANET_CENTER + to_player.normalize_or_zero() * camera_distance;

    let current = camera_transform.translation.xy();
    let smoothed = current.lerp(camera_target, 0.05);
    camera_transform.translation.x = smoothed.x;
    camera_transform.translation.y = smoothed.y;
}

// ==================== Planet Gravity Settings UI ====================

/// Extra settings UI for planet-specific configuration.
/// This adds a "Planet Gravity" section to the Controller Settings window.
fn planet_gravity_settings_ui(
    mut contexts: EguiContexts,
    mut planet_config: ResMut<PlanetConfig>,
    ui_state: Res<CharacterControllerUiState>,
) {
    // Skip if panels are hidden or not yet initialized
    if ui_state.frame_count <= 2 || !ui_state.show_panels {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    // Planet-specific settings in a separate window
    egui::Window::new("Planet Settings")
        .default_pos([10.0, 500.0])
        .default_width(300.0)
        .collapsible(true)
        .show(ctx, |ui| {
            ui.heading("Planet Gravity");
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Gravity Strength:");
                ui.add(
                    egui::DragValue::new(&mut planet_config.gravity_strength)
                        .speed(1.0)
                        .range(0.0..=500.0),
                );
            });
            ui.label(format!(
                "Current: {:.2} px/s² ({:.2} m/s²)",
                planet_config.gravity_strength,
                planet_config.gravity_strength / PIXELS_PER_METER
            ));
            ui.add_space(4.0);
            if ui.button("Reset Planet Gravity").clicked() {
                planet_config.gravity_strength = GRAVITY_STRENGTH;
            }
        });
}
