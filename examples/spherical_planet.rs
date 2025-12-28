//! Spherical Planet Example
//!
//! A playable example with a character walking on a spherical planet featuring:
//! - A circular planet with radial gravity
//! - A platform on the planet surface
//! - A triangle slope structure
//! - Dynamic orientation that adjusts to the planet's surface
//!
//! ## Controls
//! - **A/D** or **Left/Right**: Move horizontally
//! - **W/Up**: Jump
//! - **Space** (hold): Propulsion (fly up relative to planet)
//! - **S/Down** (hold): Propulsion (fly down toward planet)
//!
//! The camera follows the player and the character's "up" direction
//! always points away from the planet center.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use bevy_egui::input::EguiWantsInput;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;

// ==================== Constants ====================

const PLAYER_SIZE: f32 = 16.0;
const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

const PLANET_RADIUS: f32 = 300.0;
const PLANET_CENTER: Vec2 = Vec2::ZERO;

const PLATFORM_WIDTH: f32 = 80.0;
const PLATFORM_HEIGHT: f32 = 15.0;

const GRAVITY_STRENGTH: f32 = 600.0;

// ==================== Components ====================

/// Marker for the player entity.
#[derive(Component)]
struct Player;

/// Marker for entities affected by planetary gravity.
#[derive(Component)]
struct AffectedByPlanetaryGravity;

/// Component storing the planet center for gravity calculations.
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
                title: "Spherical Planet - Character Controller Example".into(),
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
        // Egui for settings UI
        .add_plugins(EguiPlugin::default())
        // Resources
        .init_resource::<PlanetConfig>()
        // Systems
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            // Run orientation and gravity updates before controller systems
            (update_player_orientation, apply_planetary_gravity)
                .before(msg_character_controller::systems::apply_floating_spring::<Rapier2dBackend>),
        )
        .add_systems(Update, (handle_input, camera_follow))
        .add_systems(EguiPrimaryContextPass, settings_ui)
        .run();
}

// ==================== Setup ====================

fn setup(mut commands: Commands) {
    // Camera
    commands.spawn((Camera2d, Transform::from_xyz(0.0, PLANET_RADIUS, 0.0)));

    // Spawn planet
    spawn_planet(&mut commands);

    // Spawn platform on top of planet
    spawn_surface_platform(&mut commands, 0.0, PLATFORM_WIDTH, PLATFORM_HEIGHT);

    // Spawn slope structure on the side
    spawn_surface_slope(&mut commands, std::f32::consts::FRAC_PI_4);

    // Spawn player on top of planet
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
        Pickable::IGNORE, // Prevent this UI element from blocking mouse clicks
    ));

    // Planet info
    commands.spawn((
        Text::new("Walking on a spherical planet with radial gravity"),
        TextFont {
            font_size: 18.0,
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
}

fn spawn_planet(commands: &mut Commands) {
    // Create planet as a circle collider (ball)
    // This is simpler and more efficient than a trimesh for a circle
    commands.spawn((
        Transform::from_translation(PLANET_CENTER.extend(0.0)),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::ball(PLANET_RADIUS),
        Sprite {
            color: Color::srgb(0.25, 0.35, 0.25),
            custom_size: Some(Vec2::splat(PLANET_RADIUS * 2.0)),
            ..default()
        },
    ));
}

/// Spawn a platform on the planet surface at a given angle (radians from top).
fn spawn_surface_platform(commands: &mut Commands, angle_offset: f32, width: f32, height: f32) {
    // Angle 0 = top of planet, positive = clockwise
    let surface_angle = std::f32::consts::FRAC_PI_2 - angle_offset;
    let direction = Vec2::new(surface_angle.cos(), surface_angle.sin());
    let position = PLANET_CENTER + direction * (PLANET_RADIUS + height / 2.0 + 20.0);

    // Rotation to align with surface (tangent)
    let rotation = Quat::from_rotation_z(surface_angle - std::f32::consts::FRAC_PI_2);

    commands.spawn((
        Transform::from_translation(position.extend(0.0)).with_rotation(rotation),
        GlobalTransform::default(),
        RigidBody::Fixed,
        Collider::cuboid(width / 2.0, height / 2.0),
        Sprite {
            color: Color::srgb(0.4, 0.5, 0.3),
            custom_size: Some(Vec2::new(width, height)),
            ..default()
        },
    ));
}

/// Spawn a triangular slope on the planet surface.
fn spawn_surface_slope(commands: &mut Commands, angle_offset: f32) {
    // Place slope on the surface
    let surface_angle = std::f32::consts::FRAC_PI_2 - angle_offset;
    let direction = Vec2::new(surface_angle.cos(), surface_angle.sin());
    let position = PLANET_CENTER + direction * (PLANET_RADIUS + 25.0);

    // Rotation to align with surface
    let rotation = Quat::from_rotation_z(surface_angle - std::f32::consts::FRAC_PI_2);

    // Triangle vertices in local space
    let vertices = vec![
        Vec2::new(-40.0, 0.0), // Bottom left
        Vec2::new(40.0, 0.0),  // Bottom right
        Vec2::new(40.0, 50.0), // Top right
    ];

    // Use convex_hull which works well for triangles
    let collider = Collider::convex_hull(&vertices).expect("Failed to create slope collider");

    commands.spawn((
        Transform::from_translation(position.extend(0.0)).with_rotation(rotation),
        GlobalTransform::default(),
        RigidBody::Fixed,
        collider,
        Sprite {
            color: Color::srgb(0.5, 0.4, 0.3),
            custom_size: Some(Vec2::new(80.0, 50.0)),
            ..default()
        },
    ));
}

fn spawn_player(commands: &mut Commands) {
    // Spawn on top of the planet
    let spawn_angle = std::f32::consts::FRAC_PI_2; // Top of planet
    let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
    let spawn_pos = PLANET_CENTER + direction * (PLANET_RADIUS + 50.0);

    // Initial orientation pointing away from planet
    let initial_orientation = CharacterOrientation::new(direction);

    commands
        .spawn((
            Player,
            AffectedByPlanetaryGravity,
            Transform::from_translation(spawn_pos.extend(1.0)),
            GlobalTransform::default(),
            Sprite {
                color: Color::srgb(0.2, 0.6, 0.9),
                custom_size: Some(Vec2::new(PLAYER_RADIUS * 2.0, PLAYER_SIZE)),
                ..default()
            },
        ))
        .insert((
            // Character controller
            CharacterController::new(),
            ControllerConfig::player()
                .with_float_height(PLAYER_HALF_HEIGHT)
                .with_ground_cast_width(PLAYER_RADIUS)
                .with_upright_torque_enabled(false), // We handle rotation via orientation
            initial_orientation,
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
            Collider::capsule_y(PLAYER_HALF_HEIGHT / 2.0, PLAYER_RADIUS),
            GravityScale(0.0), // We apply gravity manually
            Damping {
                linear_damping: 0.0,
                angular_damping: 5.0, // Some angular damping for stability
            },
        ));
}

// ==================== Input Handling ====================

fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    egui_wants_input: Res<EguiWantsInput>,
    mut query: Query<(&mut WalkIntent, &mut PropulsionIntent, &mut JumpRequest), With<Player>>,
) {
    // Skip input handling if egui wants keyboard input
    if egui_wants_input.wants_any_keyboard_input() {
        // Clear any ongoing movement when egui takes focus
        for (mut walk_intent, mut propulsion, _) in &mut query {
            walk_intent.set(0.0);
            propulsion.set(0.0);
        }
        return;
    }

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

// ==================== Planetary Systems ====================

/// Updates the player's orientation to point away from the planet center.
fn update_player_orientation(
    planet: Res<PlanetConfig>,
    mut query: Query<(&mut Transform, &mut CharacterOrientation), With<AffectedByPlanetaryGravity>>,
) {
    for (mut transform, mut orientation) in &mut query {
        let position = transform.translation.xy();
        let to_player = position - planet.center;
        let new_up = to_player.normalize_or_zero();

        if new_up != Vec2::ZERO {
            orientation.set_up(new_up);

            // Also rotate the transform to visually match orientation
            let angle = new_up.to_angle() - std::f32::consts::FRAC_PI_2;
            transform.rotation = Quat::from_rotation_z(angle);
        }
    }
}

/// Applies radial gravity toward the planet center.
fn apply_planetary_gravity(
    planet: Res<PlanetConfig>,
    time: Res<Time<Fixed>>,
    mut query: Query<
        (&Transform, &mut Velocity, Option<&Grounded>),
        With<AffectedByPlanetaryGravity>,
    >,
) {
    let dt = time.delta_secs();

    for (transform, mut velocity, grounded) in &mut query {
        // Only apply gravity when not grounded
        if grounded.is_none() {
            let position = transform.translation.xy();
            let to_center = planet.center - position;
            let gravity_dir = to_center.normalize_or_zero();
            let gravity = gravity_dir * planet.gravity_strength;

            velocity.linvel += gravity * dt;
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

    // Smooth camera follow - position slightly behind/above player relative to planet
    let player_pos = player_transform.translation.xy();
    let to_player = player_pos - PLANET_CENTER;
    let camera_distance = to_player.length() + 150.0; // Keep camera behind player

    let camera_target = PLANET_CENTER + to_player.normalize_or_zero() * camera_distance;

    let current = camera_transform.translation.xy();
    let smoothed = current.lerp(camera_target, 0.05);
    camera_transform.translation.x = smoothed.x;
    camera_transform.translation.y = smoothed.y;
}

// ==================== Settings UI ====================

fn settings_ui(
    mut contexts: EguiContexts,
    mut query: Query<&mut ControllerConfig, With<Player>>,
    mut frame_count: Local<u32>,
) {
    let Ok(mut config) = query.single_mut() else {
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
                if ui.button("⟳ Reset to Defaults").clicked() {
                    *config = ControllerConfig::player()
                        .with_float_height(PLAYER_HALF_HEIGHT)
                        .with_ground_cast_width(PLAYER_RADIUS)
                        .with_upright_torque_enabled(false);
                }
                ui.add_space(8.0);

                // Float Settings
                ui.collapsing("Float Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Float Height:");
                        ui.add(egui::DragValue::new(&mut config.float_height).speed(0.1).range(0.0..=100.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Cling Distance:");
                        ui.add(egui::DragValue::new(&mut config.cling_distance).speed(0.1).range(0.0..=50.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Cling Strength:");
                        ui.add(egui::DragValue::new(&mut config.cling_strength).speed(0.01).range(0.0..=2.0));
                    });
                });

                // Spring Settings
                ui.collapsing("Spring Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Spring Strength:");
                        ui.add(egui::DragValue::new(&mut config.spring_strength).speed(100.0).range(0.0..=50000.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Spring Damping:");
                        ui.add(egui::DragValue::new(&mut config.spring_damping).speed(10.0).range(0.0..=2000.0));
                    });
                });

                // Movement Settings
                ui.collapsing("Movement Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Max Speed:");
                        ui.add(egui::DragValue::new(&mut config.max_speed).speed(1.0).range(0.0..=1000.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Acceleration:");
                        ui.add(egui::DragValue::new(&mut config.acceleration).speed(10.0).range(0.0..=5000.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Friction:");
                        ui.add(egui::DragValue::new(&mut config.friction).speed(0.01).range(0.0..=1.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Air Control:");
                        ui.add(egui::DragValue::new(&mut config.air_control).speed(0.01).range(0.0..=1.0));
                    });
                });

                // Slope Settings
                ui.collapsing("Slope Settings", |ui| {
                    let mut angle_deg = config.max_slope_angle.to_degrees();
                    ui.horizontal(|ui| {
                        ui.label("Max Slope Angle (°):");
                        if ui.add(egui::DragValue::new(&mut angle_deg).speed(1.0).range(0.0..=90.0)).changed() {
                            config.max_slope_angle = angle_deg.to_radians();
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Uphill Gravity Mult:");
                        ui.add(egui::DragValue::new(&mut config.uphill_gravity_multiplier).speed(0.1).range(0.0..=5.0));
                    });
                });

                // Sensor Settings
                ui.collapsing("Sensor Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Ground Cast Mult:");
                        ui.add(egui::DragValue::new(&mut config.ground_cast_multiplier).speed(0.1).range(1.0..=20.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Ground Cast Width:");
                        ui.add(egui::DragValue::new(&mut config.ground_cast_width).speed(0.1).range(0.0..=50.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Wall Cast Mult:");
                        ui.add(egui::DragValue::new(&mut config.wall_cast_multiplier).speed(0.1).range(0.0..=5.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Wall Cast Height:");
                        ui.add(egui::DragValue::new(&mut config.wall_cast_height).speed(0.1).range(0.0..=50.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Ceiling Cast Mult:");
                        ui.add(egui::DragValue::new(&mut config.ceiling_cast_multiplier).speed(0.1).range(0.0..=10.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Ceiling Cast Width:");
                        ui.add(egui::DragValue::new(&mut config.ceiling_cast_width).speed(0.1).range(0.0..=50.0));
                    });
                });

                // Jump Settings
                ui.collapsing("Jump Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Jump Speed:");
                        ui.add(egui::DragValue::new(&mut config.jump_speed).speed(100.0).range(0.0..=20000.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Coyote Time:");
                        ui.add(egui::DragValue::new(&mut config.coyote_time).speed(0.01).range(0.0..=1.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Jump Buffer Time:");
                        ui.add(egui::DragValue::new(&mut config.jump_buffer_time).speed(0.01).range(0.0..=1.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Extra Fall Gravity:");
                        ui.add(egui::DragValue::new(&mut config.extra_fall_gravity).speed(0.1).range(0.0..=10.0));
                    });
                });

                // Upright Torque Settings
                ui.collapsing("Upright Torque Settings", |ui| {
                    ui.checkbox(&mut config.upright_torque_enabled, "Enabled");
                    ui.horizontal(|ui| {
                        ui.label("Torque Strength:");
                        ui.add(egui::DragValue::new(&mut config.upright_torque_strength).speed(10.0).range(0.0..=1000.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Torque Damping:");
                        ui.add(egui::DragValue::new(&mut config.upright_torque_damping).speed(1.0).range(0.0..=200.0));
                    });
                    let mut has_target = config.upright_target_angle.is_some();
                    let mut target_deg = config.upright_target_angle.unwrap_or(0.0).to_degrees();
                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut has_target, "Target Angle:").changed() {
                            config.upright_target_angle = if has_target { Some(target_deg.to_radians()) } else { None };
                        }
                        if has_target {
                            if ui.add(egui::DragValue::new(&mut target_deg).speed(1.0).range(-180.0..=180.0)).changed() {
                                config.upright_target_angle = Some(target_deg.to_radians());
                            }
                        }
                    });
                });
            });
        });
}
