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
use bevy_egui::EguiPlugin;
use bevy_rapier2d::prelude::*;
use helpers::{CharacterControllerUiPlugin, ControlsPlugin, Player};
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

// ==================== Components ====================

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
        // Controls (input handling and camera follow)
        .add_plugins(ControlsPlugin::default())
        // Egui for settings UI
        .add_plugins(EguiPlugin::default())
        // Character controller UI panels
        .add_plugins(CharacterControllerUiPlugin::<Player>::default())
        // Systems
        .add_systems(Startup, setup)
        .add_systems(Update, apply_gravity)
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
                custom_size: Some(Vec2::new(PLAYER_RADIUS * 2.0, (PLAYER_RADIUS + PLAYER_HALF_HEIGHT) * 2.0)),
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
            MovementIntent::default(),
        ))
        .insert((
            // Physics
            RigidBody::Dynamic,
            Velocity::default(),
            ExternalForce::default(),
            ExternalImpulse::default(),
            Collider::capsule_y(PLAYER_HALF_HEIGHT, PLAYER_RADIUS),
            GravityScale(0.0), // We apply gravity manually
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
        ));
}

// ==================== Gravity System ====================

/// Applies gravity to entities with the AffectedByGravity component.
/// This is separate from the character controller's floating spring system.
fn apply_gravity(
    time: Res<Time<Fixed>>,
    mut query: Query<
        (&CharacterController, &ControllerConfig, &mut Velocity),
        With<AffectedByGravity>,
    >,
) {
    let dt = time.delta_secs();

    for (controller, config, mut velocity) in &mut query {
        // Only apply gravity when not grounded
        if !controller.is_grounded(config) {
            velocity.linvel += controller.gravity * dt;
        }
    }
}

// ==================== Settings UI ====================

/// State for the settings UI
struct UiState {
    frame_count: u32,
    show_settings: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            frame_count: 0,
            show_settings: true, // Start with settings visible
        }
    }
}

fn settings_ui(
    mut contexts: EguiContexts,
    mut config_query: Query<
        (
            &mut ControllerConfig,
            &mut CharacterController,
            &mut Transform,
            &mut Velocity,
            &mut ExternalImpulse,
            &mut ExternalForce,
            &mut MovementIntent,
            &mut JumpRequest,
        ),
        With<Player>,
    >,
    diagnostics_query: Query<
        (
            &ControllerConfig,
            &CharacterController,
            &Transform,
            &Velocity,
            Option<&MovementIntent>,
            Option<&Grounded>,
            Option<&TouchingWall>,
            Option<&TouchingCeiling>,
        ),
        With<Player>,
    >,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut ui_state: Local<UiState>,
) {
    let Ok((
        mut config,
        mut controller,
        mut transform,
        mut velocity,
        mut external_impulse,
        mut external_force,
        mut movement_intent,
        mut jump_request,
    )) = config_query.single_mut()
    else {
        return;
    };

    // Increment frame counter
    ui_state.frame_count += 1;

    // Skip the first few frames to ensure egui is fully initialized
    if ui_state.frame_count <= 2 {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    // Toggle settings window with TAB key
    if keyboard.just_pressed(KeyCode::Tab) {
        ui_state.show_settings = !ui_state.show_settings;
    }

    // Always show the help text
    egui::Area::new(egui::Id::new("info_area"))
        .fixed_pos(egui::pos2(10.0, 40.0))
        .show(ctx, |ui| {
            ui.colored_label(
                egui::Color32::from_rgb(200, 200, 200),
                if ui_state.show_settings {
                    "Press TAB to hide panels"
                } else {
                    "Press TAB to show panels"
                },
            );
        });

    if !ui_state.show_settings {
        return;
    }

    // Controller Settings window
    egui::Window::new("Controller Settings")
        .default_pos([10.0, 80.0])
        .default_width(300.0)
        .default_height(400.0)
        .collapsible(true)
        .resizable(true)
        .show(ctx, |ui| {
            // Reload button at the top
            ui.horizontal(|ui| {
                if ui.button("Reset to Defaults").clicked() {
                    *config = ControllerConfig::player()
                        .with_float_height(15.0)
                        .with_ground_cast_width(PLAYER_RADIUS);
                    controller.gravity = Vec2::new(0.0, -980.0);
                }
                if ui.button("Respawn").clicked() {
                    // Reset position to spawn point
                    let spawn_pos = Vec2::new(-200.0, -BOX_HEIGHT / 2.0 + WALL_THICKNESS + 50.0);
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

                    // Reset movement intent and jump request
                    movement_intent.clear();
                    jump_request.reset();
                }
            });
            ui.add_space(8.0);

            config_panel_ui(ui, &mut config, &mut controller);
        });
}

fn diagnostics_ui(
    mut contexts: EguiContexts,
    diagnostics_query: Query<
        (
            &ControllerConfig,
            &CharacterController,
            &Transform,
            &Velocity,
            Option<&MovementIntent>,
            Option<&JumpRequest>,
        ),
        With<Player>,
    >,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut ui_state: Local<UiState>,
) {
    // Increment frame counter
    ui_state.frame_count += 1;

    // Skip the first few frames to ensure egui is fully initialized
    if ui_state.frame_count <= 2 {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    // Toggle settings window with TAB key
    if keyboard.just_pressed(KeyCode::Tab) {
        ui_state.show_settings = !ui_state.show_settings;
    }

    if !ui_state.show_settings {
        return;
    }

    // Diagnostics window
    if let Ok((
        config_ref,
        controller_ref,
        transform_ref,
        velocity_ref,
        movement,
        jump,
        grounded,
        wall,
        ceiling,
        ext_force,
        ext_impulse,
    )) = diagnostics_query.single()
    {
        egui::Window::new("Diagnostics")
            .default_pos([320.0, 80.0])
            .default_width(280.0)
            .default_height(400.0)
            .collapsible(true)
            .resizable(true)
            .show(ctx, |ui| {
                let data = DiagnosticsData {
                    controller: controller_ref,
                    config: config_ref,
                    transform: transform_ref,
                    velocity: velocity_ref,
                    movement_intent: movement,
                    grounded: grounded.is_some(),
                    touching_wall: wall,
                    touching_ceiling: ceiling,
                };
                diagnostics_panel_ui(ui, &data);
            });
    }
}
