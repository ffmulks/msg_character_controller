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
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
// Import the input checking resource
use bevy_egui::input::EguiWantsInput;
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
        // Egui for settings UI
        .add_plugins(EguiPlugin::default())
        // Systems
        .add_systems(Startup, setup)
        .add_systems(Update, (handle_input, apply_gravity, camera_follow))
        .add_systems(EguiPrimaryContextPass, settings_ui)
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
            MovementIntent::default(),
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
    egui_wants_input: Res<EguiWantsInput>,
    mut query: Query<(&mut MovementIntent, &mut JumpRequest), With<Player>>,
) {
    // Skip input handling if egui wants keyboard input
    if egui_wants_input.wants_any_keyboard_input() {
        // Clear any ongoing movement when egui takes focus
        for (mut movement, _) in &mut query {
            movement.clear();
        }
        return;
    }

    for (mut movement, mut jump_request) in &mut query {
        // Horizontal input (A/D or Left/Right)
        let mut horizontal = 0.0;
        if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
            horizontal -= 1.0;
        }
        if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
            horizontal += 1.0;
        }

        movement.set_walk(horizontal);

        // Vertical propulsion (Space = up, S/Down = down)
        let mut vertical = 0.0;
        if keyboard.pressed(KeyCode::Space) {
            vertical += 1.0;
        }
        if keyboard.pressed(KeyCode::KeyS) || keyboard.pressed(KeyCode::ArrowDown) {
            vertical -= 1.0;
        }
        movement.set_fly(vertical);

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
    mut query: Query<(&mut ControllerConfig, &mut CharacterController, &mut Transform, &mut Velocity), With<Player>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut ui_state: Local<UiState>,
) {
    let Ok((mut config, mut controller, mut transform, mut velocity)) = query.single_mut() else {
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
                    "Press TAB to hide settings | Settings window is VISIBLE"
                } else {
                    "Press TAB to show settings | Settings window is HIDDEN"
                },
            );
        });

    if !ui_state.show_settings {
        return;
    }

    egui::Window::new("Controller Settings")
        .default_pos([400.0, 100.0])
        .default_width(350.0)
        .default_height(500.0)
        .collapsible(true)
        .resizable(true)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                // Reload button at the top
                ui.horizontal(|ui| {
                    if ui.button("⟳ Reset to Defaults").clicked() {
                        *config = ControllerConfig::player()
                            .with_float_height(15.0)
                            .with_ground_cast_width(PLAYER_RADIUS);
                        controller.gravity = Vec2::new(0.0, -980.0);
                    }
                    if ui.button("🔄 Respawn Player").clicked() {
                        // Reset position to spawn point
                        let spawn_pos = Vec2::new(-200.0, -BOX_HEIGHT / 2.0 + WALL_THICKNESS + 50.0);
                        transform.translation = spawn_pos.extend(1.0);
                        velocity.linvel = Vec2::ZERO;
                        velocity.angvel = 0.0;
                    }
                });
                ui.add_space(8.0);

                // Gravity Settings (stored in CharacterController, takes effect immediately)
                ui.collapsing("Gravity Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Gravity X:");
                        ui.add(
                            egui::DragValue::new(&mut controller.gravity.x)
                                .speed(10.0)
                                .range(-2000.0..=2000.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Gravity Y:");
                        ui.add(
                            egui::DragValue::new(&mut controller.gravity.y)
                                .speed(10.0)
                                .range(-2000.0..=2000.0),
                        );
                    });
                });

                // Float Settings
                ui.collapsing("Float Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Float Height:");
                        ui.add(
                            egui::DragValue::new(&mut config.float_height)
                                .speed(0.1)
                                .range(0.0..=100.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Cling Distance:");
                        ui.add(
                            egui::DragValue::new(&mut config.cling_distance)
                                .speed(0.1)
                                .range(0.0..=50.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Cling Strength:");
                        ui.add(
                            egui::DragValue::new(&mut config.cling_strength)
                                .speed(0.01)
                                .range(0.0..=2.0),
                        );
                    });
                });

                // Spring Settings
                ui.collapsing("Spring Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Spring Strength:");
                        ui.add(
                            egui::DragValue::new(&mut config.spring_strength)
                                .speed(100.0)
                                .range(0.0..=50000.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Spring Damping:");
                        ui.add(
                            egui::DragValue::new(&mut config.spring_damping)
                                .speed(10.0)
                                .range(0.0..=2000.0),
                        );
                    });
                });

                // Movement Settings
                ui.collapsing("Movement Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Max Speed:");
                        ui.add(
                            egui::DragValue::new(&mut config.max_speed)
                                .speed(1.0)
                                .range(0.0..=1000.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Acceleration:");
                        ui.add(
                            egui::DragValue::new(&mut config.acceleration)
                                .speed(10.0)
                                .range(0.0..=5000.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Friction:");
                        ui.add(
                            egui::DragValue::new(&mut config.friction)
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Air Control:");
                        ui.add(
                            egui::DragValue::new(&mut config.air_control)
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });
                });

                // Slope Settings
                ui.collapsing("Slope Settings", |ui| {
                    let mut angle_deg = config.max_slope_angle.to_degrees();
                    ui.horizontal(|ui| {
                        ui.label("Max Slope Angle (°):");
                        if ui
                            .add(
                                egui::DragValue::new(&mut angle_deg)
                                    .speed(1.0)
                                    .range(0.0..=90.0),
                            )
                            .changed()
                        {
                            config.max_slope_angle = angle_deg.to_radians();
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Uphill Gravity Mult:");
                        ui.add(
                            egui::DragValue::new(&mut config.uphill_gravity_multiplier)
                                .speed(0.1)
                                .range(0.0..=5.0),
                        );
                    });
                });

                // Sensor Settings
                ui.collapsing("Sensor Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Ground Cast Mult:");
                        ui.add(
                            egui::DragValue::new(&mut config.ground_cast_multiplier)
                                .speed(0.1)
                                .range(1.0..=20.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Ground Cast Width:");
                        ui.add(
                            egui::DragValue::new(&mut config.ground_cast_width)
                                .speed(0.1)
                                .range(0.0..=50.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Wall Cast Mult:");
                        ui.add(
                            egui::DragValue::new(&mut config.wall_cast_multiplier)
                                .speed(0.1)
                                .range(0.0..=5.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Wall Cast Height:");
                        ui.add(
                            egui::DragValue::new(&mut config.wall_cast_height)
                                .speed(0.1)
                                .range(0.0..=50.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Ceiling Cast Mult:");
                        ui.add(
                            egui::DragValue::new(&mut config.ceiling_cast_multiplier)
                                .speed(0.1)
                                .range(0.0..=10.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Ceiling Cast Width:");
                        ui.add(
                            egui::DragValue::new(&mut config.ceiling_cast_width)
                                .speed(0.1)
                                .range(0.0..=50.0),
                        );
                    });
                });

                // Jump Settings
                ui.collapsing("Jump Settings", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Jump Speed:");
                        ui.add(
                            egui::DragValue::new(&mut config.jump_speed)
                                .speed(100.0)
                                .range(0.0..=20000.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Coyote Time:");
                        ui.add(
                            egui::DragValue::new(&mut config.coyote_time)
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Jump Buffer Time:");
                        ui.add(
                            egui::DragValue::new(&mut config.jump_buffer_time)
                                .speed(0.01)
                                .range(0.0..=1.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Extra Fall Gravity:");
                        ui.add(
                            egui::DragValue::new(&mut config.extra_fall_gravity)
                                .speed(0.1)
                                .range(0.0..=10.0),
                        );
                    });
                });

                // Upright Torque Settings
                ui.collapsing("Upright Torque Settings", |ui| {
                    ui.checkbox(&mut config.upright_torque_enabled, "Enabled");
                    ui.horizontal(|ui| {
                        ui.label("Torque Strength:");
                        ui.add(
                            egui::DragValue::new(&mut config.upright_torque_strength)
                                .speed(10.0)
                                .range(0.0..=1000.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.label("Torque Damping:");
                        ui.add(
                            egui::DragValue::new(&mut config.upright_torque_damping)
                                .speed(1.0)
                                .range(0.0..=200.0),
                        );
                    });
                    // upright_target_angle is Option<f32> - handle separately
                    let mut has_target = config.upright_target_angle.is_some();
                    let mut target_deg = config.upright_target_angle.unwrap_or(0.0).to_degrees();
                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut has_target, "Target Angle:").changed() {
                            config.upright_target_angle = if has_target {
                                Some(target_deg.to_radians())
                            } else {
                                None
                            };
                        }
                        if has_target {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut target_deg)
                                        .speed(1.0)
                                        .range(-180.0..=180.0),
                                )
                                .changed()
                            {
                                config.upright_target_angle = Some(target_deg.to_radians());
                            }
                        }
                    });
                });
            });
        });
}
