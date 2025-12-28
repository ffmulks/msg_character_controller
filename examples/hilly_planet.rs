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

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use bevy_egui::input::EguiWantsInput;
use bevy_rapier2d::prelude::*;
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

/// Marker for the player entity.
#[derive(Component)]
struct Player;

/// Marker for entities affected by planetary gravity.
#[derive(Component)]
struct AffectedByPlanetaryGravity;

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
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(PIXELS_PER_METER))
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
            (
                update_player_orientation,
                apply_planetary_gravity, // Gravity always applies (walking AND flying)
            ),
        )
        .add_systems(Update, (handle_input, camera_follow))
        .add_systems(EguiPrimaryContextPass, settings_ui)
        .run();
}

// ==================== Setup ====================

fn setup(mut commands: Commands) {
    // Camera
    commands.spawn((Camera2d, Transform::from_xyz(0.0, PLANET_BASE_RADIUS + 100.0, 0.0)));

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
            let angle = new_up.to_angle() - PI / 2.0;
            transform.rotation = Quat::from_rotation_z(angle);
        }
    }
}

/// Applies radial gravity toward the planet center.
/// Gravity is applied when the character is not grounded.
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
        // Apply gravity when not grounded
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
    mut query: Query<(&mut ControllerConfig, &mut Transform, &mut Velocity), With<Player>>,
    mut planet_config: ResMut<PlanetConfig>,
    mut frame_count: Local<u32>,
) {
    let Ok((mut config, mut transform, mut velocity)) = query.single_mut() else {
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
                    if ui.button("⟳ Reset to Defaults").clicked() {
                        *config = ControllerConfig::player()
                            .with_float_height(PLAYER_HALF_HEIGHT)
                            .with_ground_cast_width(PLAYER_RADIUS)
                            .with_upright_torque_enabled(false);
                        planet_config.gravity_strength = GRAVITY_STRENGTH;
                    }
                    if ui.button("🔄 Respawn Player").clicked() {
                        // Reset position to top of planet
                        let spawn_angle = PI / 2.0;
                        let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
                        let surface_radius = planet_radius_at_angle(spawn_angle);
                        let spawn_pos = PLANET_CENTER + direction * (surface_radius + 40.0);
                        transform.translation = spawn_pos.extend(1.0);
                        transform.rotation = Quat::IDENTITY;
                        velocity.linvel = Vec2::ZERO;
                        velocity.angvel = 0.0;
                    }
                });
                ui.add_space(8.0);

                // Planet Gravity Settings (takes effect immediately)
                ui.collapsing("Planet Gravity", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Gravity Strength:");
                        ui.add(
                            egui::DragValue::new(&mut planet_config.gravity_strength)
                                .speed(1.0)
                                .range(0.0..=500.0),
                        );
                    });
                    ui.label(format!("Current: {:.2} px/s² ({:.2} m/s²)",
                        planet_config.gravity_strength,
                        planet_config.gravity_strength / PIXELS_PER_METER));
                });

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
