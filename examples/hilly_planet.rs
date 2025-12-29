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
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use bevy_rapier2d::prelude::*;
use helpers::{
    ControlsPlugin, Player, float_settings_ui, jump_settings_ui, movement_settings_ui,
    sensor_settings_ui, slope_settings_ui, spring_settings_ui, upright_torque_settings_ui,
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
        // Systems
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            // Update orientation and gravity before controller systems
            update_player_orientation_and_gravity.before(
                msg_character_controller::systems::apply_floating_spring::<Rapier2dBackend>,
            ),
        )
        .add_systems(Update, camera_follow)
        .add_systems(EguiPrimaryContextPass, settings_ui)
        .run();
}

// ==================== Setup ====================

fn setup(mut commands: Commands) {
    // Camera
    commands.spawn((
        Camera2d,
        Transform::from_xyz(0.0, PLANET_BASE_RADIUS + 100.0, 0.0),
    ));

    // Spawn hilly planet
    spawn_hilly_planet(&mut commands);

    // Spawn player on top of planet
    spawn_player(&mut commands);

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

/// Generate the radius at a given angle with hills.
fn planet_radius_at_angle(angle: f32) -> f32 {
    // Create multiple overlapping sine waves for interesting terrain
    let hill1 = (angle * HILL_FREQUENCY as f32).sin() * HILL_AMPLITUDE;
    let hill2 = (angle * (HILL_FREQUENCY as f32 * 2.3)).sin() * (HILL_AMPLITUDE * 0.4);
    let hill3 = (angle * (HILL_FREQUENCY as f32 * 0.5)).sin() * (HILL_AMPLITUDE * 0.6);

    PLANET_BASE_RADIUS + hill1 + hill2 + hill3
}

fn spawn_hilly_planet(commands: &mut Commands) {
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

    // Create a visual mesh for the planet (using a sprite for simplicity)
    // The debug render will show the actual collision shape
    commands.spawn((
        Transform::from_translation(PLANET_CENTER.extend(-1.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        collider,
        Sprite {
            color: Color::srgb(0.2, 0.35, 0.2),
            custom_size: Some(Vec2::splat(PLANET_BASE_RADIUS * 2.0 + HILL_AMPLITUDE * 4.0)),
            ..default()
        },
    ));

    // Add some distinct landmark features on the surface

    // A flat platform area (good landing spot)
    spawn_surface_platform(commands, PI * 0.5, 60.0, 8.0); // Top

    // A ramp/slope structure
    spawn_surface_slope(commands, PI * 0.25); // Upper right

    // Another platform on the side
    spawn_surface_platform(commands, PI, 50.0, 6.0); // Left side

    // Small stepping stones
    spawn_surface_platform(commands, PI * 1.5, 30.0, 5.0); // Bottom
    spawn_surface_platform(commands, PI * 1.75, 25.0, 5.0); // Bottom right
}

/// Spawn a platform on the planet surface at a given angle.
fn spawn_surface_platform(commands: &mut Commands, angle: f32, width: f32, height: f32) {
    let direction = Vec2::new(angle.cos(), angle.sin());
    let surface_radius = planet_radius_at_angle(angle);
    let position = PLANET_CENTER + direction * (surface_radius + height / 2.0 + 5.0);

    // Rotation to align with surface (tangent)
    let rotation = Quat::from_rotation_z(angle - PI / 2.0);

    commands.spawn((
        Transform::from_translation(position.extend(0.0)).with_rotation(rotation),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::cuboid(width / 2.0, height / 2.0),
        Sprite {
            color: Color::srgb(0.4, 0.5, 0.35),
            custom_size: Some(Vec2::new(width, height)),
            ..default()
        },
    ));
}

/// Spawn a triangular slope on the planet surface.
fn spawn_surface_slope(commands: &mut Commands, angle: f32) {
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

    commands.spawn((
        Transform::from_translation(position.extend(0.0)).with_rotation(rotation),
        GlobalTransform::default(),
        RigidBody::Fixed,
        collider,
        Sprite {
            color: Color::srgb(0.5, 0.4, 0.3),
            custom_size: Some(Vec2::new(60.0, 40.0)),
            ..default()
        },
    ));
}

fn spawn_player(commands: &mut Commands) {
    // Spawn on top of the planet
    let spawn_angle = PI / 2.0; // Top of planet
    let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
    let surface_radius = planet_radius_at_angle(spawn_angle);
    let spawn_pos = PLANET_CENTER + direction * (surface_radius + 40.0);

    // Initial orientation pointing away from planet
    let initial_orientation = CharacterOrientation::new(direction);

    // Initial gravity pointing toward planet center
    let initial_gravity = -direction * GRAVITY_STRENGTH;

    commands
        .spawn((
            Player,
            Transform::from_translation(spawn_pos.extend(1.0)),
            GlobalTransform::default(),
            Sprite {
                color: Color::srgb(0.2, 0.6, 0.9),
                custom_size: Some(Vec2::new(PLAYER_RADIUS * 2.0, PLAYER_SIZE)),
                ..default()
            },
        ))
        .insert((
            // Character controller with initial gravity pointing toward planet
            CharacterController::with_gravity(initial_gravity),
            ControllerConfig::player()
                .with_float_height(PLAYER_HALF_HEIGHT)
                .with_ground_cast_width(PLAYER_RADIUS)
                .with_upright_torque_enabled(false), // We handle rotation via orientation
            initial_orientation,
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

/// Updates the player's orientation and gravity to match the planet.
///
/// Orientation points away from the planet center (up = radially outward).
/// Gravity points toward the planet center and is stored in CharacterController.
/// The internal gravity system then applies it when not grounded.
fn update_player_orientation_and_gravity(
    planet: Res<PlanetConfig>,
    mut query: Query<
        (
            &mut Transform,
            &mut CharacterOrientation,
            &mut CharacterController,
        ),
        With<Player>,
    >,
) {
    for (mut transform, mut orientation, mut controller) in &mut query {
        let position = transform.translation.xy();
        let to_player = position - planet.center;
        let new_up = to_player.normalize_or_zero();

        if new_up != Vec2::ZERO {
            // Update orientation to point away from planet
            orientation.set_up(new_up);

            // Update gravity to point toward planet center
            controller.gravity = -new_up * planet.gravity_strength;

            // Also rotate the transform to visually match orientation
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

// ==================== Settings UI ====================

fn settings_ui(
    mut contexts: EguiContexts,
    mut query: Query<
        (
            &mut ControllerConfig,
            &mut CharacterController,
            &mut Transform,
            &mut Velocity,
            &mut ExternalImpulse,
            &mut ExternalForce,
            &mut MovementIntent,
        ),
        With<Player>,
    >,
    mut planet_config: ResMut<PlanetConfig>,
    mut frame_count: Local<u32>,
) {
    let Ok((
        mut config,
        mut controller,
        mut transform,
        mut velocity,
        mut external_impulse,
        mut external_force,
        mut movement_intent,
    )) = query.single_mut()
    else {
        return;
    };

    // Increment frame counter
    *frame_count += 1;

    // Skip the first few frames to ensure egui is fully initialized
    if *frame_count <= 2 {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::Window::new("Controller Settings")
        .default_pos([10.0, 50.0])
        .default_width(300.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                // Reload button at the top
                ui.horizontal(|ui| {
                    if ui.button("Reset to Defaults").clicked() {
                        *config = ControllerConfig::player()
                            .with_float_height(PLAYER_HALF_HEIGHT)
                            .with_ground_cast_width(PLAYER_RADIUS)
                            .with_upright_torque_enabled(false);
                        planet_config.gravity_strength = GRAVITY_STRENGTH;
                    }
                    if ui.button("Respawn Player").clicked() {
                        // Reset position to top of planet
                        let spawn_angle = PI / 2.0;
                        let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
                        let surface_radius = planet_radius_at_angle(spawn_angle);
                        let spawn_pos = PLANET_CENTER + direction * (surface_radius + 40.0);
                        transform.translation = spawn_pos.extend(1.0);
                        transform.rotation = Quat::IDENTITY;

                        // Reset velocity
                        velocity.linvel = Vec2::ZERO;
                        velocity.angvel = 0.0;

                        // Reset external impulse and force
                        external_impulse.impulse = Vec2::ZERO;
                        external_impulse.torque_impulse = 0.0;
                        external_force.force = Vec2::ZERO;
                        external_force.torque = 0.0;

                        // Reset controller state (keep gravity)
                        let gravity = controller.gravity;
                        *controller = CharacterController::with_gravity(gravity);

                        // Reset movement intent
                        movement_intent.clear();
                    }
                });
                ui.add_space(8.0);

                // Planet Gravity Settings (custom for this example)
                ui.collapsing("Planet Gravity", |ui| {
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
                });

                // Use helper functions for standard config sections
                float_settings_ui(ui, &mut config);
                spring_settings_ui(ui, &mut config);
                movement_settings_ui(ui, &mut config);
                slope_settings_ui(ui, &mut config);
                sensor_settings_ui(ui, &mut config);
                jump_settings_ui(ui, &mut config);
                upright_torque_settings_ui(ui, &mut config);
            });
        });
}
