//! Float Test Example
//!
//! This example specifically tests if the character controller's floating spring
//! system is working. It spawns a character high in the air above a platform.
//!
//! If floating works: The character should descend and hover at float_height above the platform
//! If floating doesn't work: The character will fall through or sit directly on the platform
//!
//! Controls:
//! - **WASD/Arrow Keys**: Move
//! - **Space**: Jump (to test if we can jump from floating position)

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use bevy_egui::input::EguiWantsInput;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;

const PLAYER_RADIUS: f32 = 6.0;
const PLAYER_HALF_HEIGHT: f32 = 8.0;

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
        // Egui for settings UI
        .add_plugins(EguiPlugin::default())
        // Resources
        .insert_resource(Gravity(Vec2::new(0.0, -980.0)))
        // Systems
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (handle_input, apply_gravity, debug_floating, camera_follow),
        )
        .add_systems(EguiPrimaryContextPass, settings_ui)
        .run();
}

#[derive(Resource)]
struct Gravity(Vec2);

#[derive(Component)]
struct Player;

#[derive(Component)]
struct AffectedByGravity;

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
    let spawn_pos = Vec2::new(0.0, 200.0); // 400 units above platform!

    // Calculate expected float position
    // float_height is the gap between the BOTTOM of the capsule and the ground
    let float_height = 2.0; // 2 pixels gap between capsule bottom and ground
    let collider_bottom_offset = PLAYER_HALF_HEIGHT / 2.0 + PLAYER_RADIUS; // 4 + 6 = 10
    let platform_top = -200.0 + 20.0; // Platform at y=-200, half-height=20, so top at -180
    let expected_hover_y = platform_top + float_height + collider_bottom_offset; // -180 + 2 + 10 = -168

    println!("=== FLOAT TEST SETUP ===");
    println!("Spawning player at Y: {}", spawn_pos.y);
    println!("Platform top at Y: {}", platform_top);
    println!("Float height (gap from capsule bottom to ground): {}", float_height);
    println!("Collider bottom offset: {}", collider_bottom_offset);
    println!("Expected center Y: {}", expected_hover_y);

    commands
        .spawn((
            Player,
            AffectedByGravity,
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
            ControllerConfig::player()
                .with_float_height(float_height) // Gap between capsule bottom and ground
                .with_spring(20000.0, 500.0) // VERY strong spring
                .with_ground_cast_width(PLAYER_RADIUS),
            WalkIntent::default(),
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
            GravityScale(0.0), // Manual gravity
            Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
        ));
}

fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    egui_wants_input: Res<EguiWantsInput>,
    mut query: Query<(&mut WalkIntent, &mut JumpRequest), With<Player>>,
) {
    // Skip input handling if egui wants keyboard input
    if egui_wants_input.wants_any_keyboard_input() {
        // Clear any ongoing movement when egui takes focus
        for (mut walk_intent, _) in &mut query {
            walk_intent.set(0.0);
        }
        return;
    }

    for (mut walk_intent, mut jump_request) in &mut query {
        // Horizontal movement
        let mut direction = 0.0;
        if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
            direction -= 1.0;
        }
        if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
            direction += 1.0;
        }
        walk_intent.set(direction);

        // Jump
        if keyboard.just_pressed(KeyCode::Space) {
            jump_request.request(time.elapsed_secs());
        }
    }
}

fn apply_gravity(
    gravity: Res<Gravity>,
    time: Res<Time>,
    mut query: Query<(&mut Velocity, Option<&Grounded>), With<AffectedByGravity>>,
) {
    let dt = time.delta_secs();

    for (mut velocity, grounded) in &mut query {
        // Only apply gravity when not grounded
        if grounded.is_none() {
            velocity.linvel += gravity.0 * dt;
        }
    }
}

fn debug_floating(
    mut text_query: Query<(&mut Text, &mut TextColor), With<DebugText>>,
    player_query: Query<(&Transform, &Velocity, &CharacterController, Option<&Grounded>), With<Player>>,
) {
    let Ok((transform, velocity, controller, grounded)) = player_query.single() else {
        return;
    };

    let Ok((mut text, mut color)) = text_query.single_mut() else {
        return;
    };

    let grounded_str = if grounded.is_some() { "YES" } else { "NO" };
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

    camera_transform.translation.x = player_transform.translation.x;
    camera_transform.translation.y = player_transform.translation.y;
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
                        .with_float_height(2.0)
                        .with_spring(20000.0, 500.0)
                        .with_ground_cast_width(PLAYER_RADIUS);
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
