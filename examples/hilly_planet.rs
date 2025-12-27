//! Hilly Planet Example
//!
//! A playable example with a character walking on a spherical planet with hilly terrain:
//! - Uses 16 pixels per meter units
//! - Spherical planet with procedural hilly surface
//! - Realistic gravity: 9.81 * 16 = 156.96 px/s²
//! - Flying mode does NOT disable gravity, it only changes what up/down do
//!
//! ## Controls
//! - **A/D** or **Left/Right**: Move horizontally
//! - **W/Up**: Jump (walking) / Move up relative to planet (flying)
//! - **S/Down**: Move down relative to planet (flying only)
//! - **Space** or **Shift**: Toggle between Flying and Walking modes
//!
//! In flying mode, gravity still pulls you toward the planet center,
//! but you can thrust up/down relative to your current orientation.

use bevy::prelude::*;
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

// Fly thrust strength (how fast you can fly up/down)
const FLY_THRUST: f32 = 200.0;

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
        // Resources
        .init_resource::<PlanetConfig>()
        // Systems
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            (
                update_player_orientation,
                apply_planetary_gravity, // Gravity always applies (walking AND flying)
            )
                .before(msg_character_controller::systems::update_ground_detection::<Rapier2dBackend>),
        )
        .add_systems(
            Update,
            (handle_input, toggle_mode, camera_follow, display_mode_ui),
        )
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
        Text::new("A/D or Left/Right: Move | W/Up: Jump/Fly Up | S/Down: Fly Down | Space/Shift: Toggle Mode"),
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
    ));

    // Mode display UI
    commands.spawn((
        Text::new("Mode: Walking"),
        TextFont {
            font_size: 24.0,
            ..default()
        },
        TextColor(Color::srgb(0.2, 0.8, 0.2)),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(40.0),
            left: Val::Px(10.0),
            ..default()
        },
        ModeDisplay,
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
    ));

    // Flying info
    commands.spawn((
        Text::new("Flying: Gravity still applies! Use W/S to thrust."),
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
            CharacterController::walking(),
            ControllerConfig::player()
                .with_float_height(PLAYER_HALF_HEIGHT)
                .with_ground_cast_width(PLAYER_RADIUS)
                .with_upright_torque_enabled(false), // We handle rotation via orientation
            initial_orientation,
            WalkIntent::default(),
            FlyIntent::default(),
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
    mut query: Query<
        (
            &CharacterController,
            &CharacterOrientation,
            &mut WalkIntent,
            &mut FlyIntent,
            &mut JumpRequest,
            &mut Velocity,
        ),
        With<Player>,
    >,
) {
    for (controller, orientation, mut walk_intent, mut fly_intent, mut jump_request, mut velocity) in
        &mut query
    {
        // Horizontal input (A/D or Left/Right)
        let mut horizontal = 0.0;
        if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
            horizontal -= 1.0;
        }
        if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
            horizontal += 1.0;
        }

        // Vertical input (W/S or Up/Down)
        let mut vertical = 0.0;
        if keyboard.pressed(KeyCode::KeyW) || keyboard.pressed(KeyCode::ArrowUp) {
            vertical += 1.0;
        }
        if keyboard.pressed(KeyCode::KeyS) || keyboard.pressed(KeyCode::ArrowDown) {
            vertical -= 1.0;
        }

        if controller.is_walking() {
            // Walking mode: horizontal movement + jump
            walk_intent.set(horizontal);

            // Jump on W or Up (just pressed)
            if keyboard.just_pressed(KeyCode::KeyW) || keyboard.just_pressed(KeyCode::ArrowUp) {
                jump_request.request(time.elapsed_secs());
            }
        } else {
            // Flying mode: horizontal movement via FlyIntent, vertical via direct thrust
            // This allows up/down to work against gravity
            fly_intent.set(Vec2::new(horizontal, 0.0));

            // Apply vertical thrust in the character's local up direction
            // This fights against gravity when going up, assists when going down
            if vertical != 0.0 {
                let thrust_dir = orientation.up() * vertical;
                velocity.linvel += thrust_dir * FLY_THRUST * time.delta_secs();
            }
        }
    }
}

fn toggle_mode(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut CharacterController, With<Player>>,
) {
    if keyboard.just_pressed(KeyCode::Space)
        || keyboard.just_pressed(KeyCode::ShiftLeft)
        || keyboard.just_pressed(KeyCode::ShiftRight)
    {
        for mut controller in &mut query {
            if controller.is_walking() {
                controller.mode = msg_character_controller::config::ControllerMode::Flying;
            } else {
                controller.mode = msg_character_controller::config::ControllerMode::Walking;
            }
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
/// IMPORTANT: This applies in BOTH walking AND flying modes!
/// Flying just gives you thrust to counteract gravity, it doesn't disable it.
fn apply_planetary_gravity(
    planet: Res<PlanetConfig>,
    time: Res<Time<Fixed>>,
    mut query: Query<
        (
            &Transform,
            &CharacterController,
            &mut Velocity,
            Option<&Grounded>,
        ),
        With<AffectedByPlanetaryGravity>,
    >,
) {
    let dt = time.delta_secs();

    for (transform, controller, mut velocity, grounded) in &mut query {
        // Apply gravity when:
        // - Walking mode AND not grounded (falling)
        // - Flying mode ALWAYS (gravity doesn't turn off, you have to thrust against it)
        let should_apply_gravity = if controller.is_walking() {
            grounded.is_none() // Only when airborne in walking mode
        } else {
            true // Always in flying mode
        };

        if should_apply_gravity {
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

// ==================== UI ====================

#[derive(Component)]
struct ModeDisplay;

fn display_mode_ui(
    player_query: Query<&CharacterController, With<Player>>,
    mut text_query: Query<(&mut Text, &mut TextColor), With<ModeDisplay>>,
) {
    let Ok(controller) = player_query.single() else {
        return;
    };

    for (mut text, mut color) in &mut text_query {
        if controller.is_walking() {
            **text = "Mode: Walking".to_string();
            color.0 = Color::srgb(0.2, 0.8, 0.2);
        } else {
            **text = "Mode: Flying (gravity still applies!)".to_string();
            color.0 = Color::srgb(0.9, 0.6, 0.2);
        }
    }
}
