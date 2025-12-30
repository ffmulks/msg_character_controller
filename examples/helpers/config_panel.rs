//! Configuration panel for ControllerConfig.
//!
//! Provides a reusable egui panel for editing character controller configuration.

use bevy_egui::egui;
use msg_character_controller::prelude::*;

/// Renders the gravity settings collapsible section.
pub fn gravity_settings_ui(ui: &mut egui::Ui, controller: &mut CharacterController) {
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
}

/// Renders the float settings collapsible section.
pub fn float_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
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
            ui.label("Ground Tolerance:");
            ui.add(
                egui::DragValue::new(&mut config.ground_tolerance)
                    .speed(0.1)
                    .range(0.0..=20.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Grounding Distance:");
            ui.add(
                egui::DragValue::new(&mut config.grounding_distance)
                    .speed(0.1)
                    .range(0.0..=50.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Grounding Strength:");
            ui.add(
                egui::DragValue::new(&mut config.grounding_strength)
                    .speed(0.1)
                    .range(0.1..=10.0),
            );
        });
    });
}

/// Renders the spring settings collapsible section.
pub fn spring_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
    ui.collapsing("Spring Settings", |ui| {
        ui.horizontal(|ui| {
            ui.label("Spring Strength:");
            ui.add(
                egui::DragValue::new(&mut config.spring_strength)
                    .speed(100.0)
                    .range(0.0..=1000000000.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Spring Damping:");
            ui.add(
                egui::DragValue::new(&mut config.spring_damping)
                    .speed(10.0)
                    .range(0.0..=1000000000.0),
            );
        });

        // spring_max_force is Option<f32>
        let mut has_max_force = config.spring_max_force.is_some();
        let mut max_force = config.spring_max_force.unwrap_or(3000.0);
        ui.horizontal(|ui| {
            if ui.checkbox(&mut has_max_force, "Max Force:").changed() {
                config.spring_max_force = if has_max_force { Some(max_force) } else { None };
            }
            if has_max_force {
                if ui
                    .add(
                        egui::DragValue::new(&mut max_force)
                            .speed(100.0)
                            .range(0.0..=1000000000.0),
                    )
                    .changed()
                {
                    config.spring_max_force = Some(max_force);
                }
            }
        });

        // spring_max_velocity is Option<f32>
        let mut has_max_vel = config.spring_max_velocity.is_some();
        let mut max_vel = config.spring_max_velocity.unwrap_or(100.0);
        ui.horizontal(|ui| {
            if ui.checkbox(&mut has_max_vel, "Max Velocity:").changed() {
                config.spring_max_velocity = if has_max_vel { Some(max_vel) } else { None };
            }
            if has_max_vel {
                if ui
                    .add(
                        egui::DragValue::new(&mut max_vel)
                            .speed(1.0)
                            .range(0.0..=100000.0),
                    )
                    .changed()
                {
                    config.spring_max_velocity = Some(max_vel);
                }
            }
        });
    });
}

/// Renders the movement settings collapsible section.
pub fn movement_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
    ui.collapsing("Movement Settings", |ui| {
        ui.horizontal(|ui| {
            ui.label("Max Speed:");
            ui.add(
                egui::DragValue::new(&mut config.max_speed)
                    .speed(1.0)
                    .range(0.0..=100000.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Acceleration:");
            ui.add(
                egui::DragValue::new(&mut config.acceleration)
                    .speed(10.0)
                    .range(0.0..=100000.0),
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
        ui.checkbox(&mut config.wall_clinging, "Wall Clinging");
    });
}

/// Renders the slope settings collapsible section.
pub fn slope_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
    ui.collapsing("Slope Settings", |ui| {
        let mut angle_deg = config.max_slope_angle.to_degrees();
        ui.horizontal(|ui| {
            ui.label("Max Slope Angle (deg):");
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
                    .range(0.0..=10.0),
            );
        });
    });
}

/// Renders the sensor settings collapsible section.
pub fn sensor_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
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
}

/// Renders the jump settings collapsible section.
pub fn jump_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
    ui.collapsing("Jump Settings", |ui| {
        ui.horizontal(|ui| {
            ui.label("Jump Speed:");
            ui.add(
                egui::DragValue::new(&mut config.jump_speed)
                    .speed(100.0)
                    .range(0.0..=1000000000.0),
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
                    .range(0.0..=100.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Spring Filter Duration:");
            ui.add(
                egui::DragValue::new(&mut config.jump_spring_filter_duration)
                    .speed(0.01)
                    .range(0.0..=1.0),
            );
        });
    });
}

/// Renders the wall jump settings collapsible section.
pub fn wall_jump_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
    ui.collapsing("Wall Jump Settings", |ui| {
        ui.checkbox(&mut config.wall_jumping, "Wall Jumping Enabled");
        ui.add_enabled_ui(config.wall_jumping, |ui| {
            let mut angle_deg = config.wall_jump_angle.to_degrees();
            ui.horizontal(|ui| {
                ui.label("Wall Jump Angle (deg):");
                if ui
                    .add(
                        egui::DragValue::new(&mut angle_deg)
                            .speed(1.0)
                            .range(0.0..=90.0),
                    )
                    .changed()
                {
                    config.wall_jump_angle = angle_deg.to_radians();
                }
            });
            ui.label("(0° = straight up, 45° = diagonal)");
        });
    });
}

/// Renders the upright torque settings collapsible section.
pub fn upright_torque_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
    ui.collapsing("Upright Torque Settings", |ui| {
        ui.checkbox(&mut config.upright_torque_enabled, "Enabled");
        ui.horizontal(|ui| {
            ui.label("Torque Strength:");
            ui.add(
                egui::DragValue::new(&mut config.upright_torque_strength)
                    .speed(10.0)
                    .range(0.0..=1000000.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Torque Damping:");
            ui.add(
                egui::DragValue::new(&mut config.upright_torque_damping)
                    .speed(1.0)
                    .range(0.0..=1000000.0),
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

        // upright_max_torque is Option<f32>
        let mut has_max_torque = config.upright_max_torque.is_some();
        let mut max_torque = config.upright_max_torque.unwrap_or(1_000_000.0);
        ui.horizontal(|ui| {
            if ui.checkbox(&mut has_max_torque, "Max Torque:").changed() {
                config.upright_max_torque = if has_max_torque {
                    Some(max_torque)
                } else {
                    None
                };
            }
            if has_max_torque {
                if ui
                    .add(
                        egui::DragValue::new(&mut max_torque)
                            .speed(10000.0)
                            .range(0.0..=100_000_000.0),
                    )
                    .changed()
                {
                    config.upright_max_torque = Some(max_torque);
                }
            }
        });

        // upright_max_angular_velocity is Option<f32>
        let mut has_max_vel = config.upright_max_angular_velocity.is_some();
        let mut max_vel = config.upright_max_angular_velocity.unwrap_or(10.0);
        ui.horizontal(|ui| {
            if ui.checkbox(&mut has_max_vel, "Max Angular Vel:").changed() {
                config.upright_max_angular_velocity =
                    if has_max_vel { Some(max_vel) } else { None };
            }
            if has_max_vel {
                if ui
                    .add(
                        egui::DragValue::new(&mut max_vel)
                            .speed(0.5)
                            .range(0.0..=50.0),
                    )
                    .changed()
                {
                    config.upright_max_angular_velocity = Some(max_vel);
                }
            }
        });
    });
}

/// Renders the stair climbing settings collapsible section.
pub fn stair_settings_ui(ui: &mut egui::Ui, stair_config: &mut StairConfig) {
    ui.collapsing("Stair Climbing Settings", |ui| {
        ui.checkbox(&mut stair_config.enabled, "Enabled");
        ui.horizontal(|ui| {
            ui.label("Max Climb Height:");
            ui.add(
                egui::DragValue::new(&mut stair_config.max_climb_height)
                    .speed(0.5)
                    .range(0.0..=50.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Min Step Depth:");
            ui.add(
                egui::DragValue::new(&mut stair_config.min_step_depth)
                    .speed(0.5)
                    .range(0.0..=20.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Stair Cast Width:");
            ui.add(
                egui::DragValue::new(&mut stair_config.stair_cast_width)
                    .speed(0.5)
                    .range(0.0..=20.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Stair Cast Offset:");
            ui.add(
                egui::DragValue::new(&mut stair_config.stair_cast_offset)
                    .speed(0.1)
                    .range(0.0..=10.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Stair Tolerance:");
            ui.add(
                egui::DragValue::new(&mut stair_config.stair_tolerance)
                    .speed(0.1)
                    .range(0.0..=10.0),
            );
        });
        ui.horizontal(|ui| {
            ui.label("Climb Force Mult:");
            ui.add(
                egui::DragValue::new(&mut stair_config.climb_force_multiplier)
                    .speed(0.1)
                    .range(0.0..=10000.0),
            );
        });
    });
}

/// Renders the complete config panel with all settings sections.
///
/// This is a convenience function that renders all configuration sections
/// in a single scrollable area.
pub fn config_panel_ui(
    ui: &mut egui::Ui,
    config: &mut ControllerConfig,
    controller: &mut CharacterController,
) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        gravity_settings_ui(ui, controller);
        float_settings_ui(ui, config);
        spring_settings_ui(ui, config);
        movement_settings_ui(ui, config);
        slope_settings_ui(ui, config);
        sensor_settings_ui(ui, config);
        jump_settings_ui(ui, config);
        wall_jump_settings_ui(ui, config);
        upright_torque_settings_ui(ui, config);
        if let Some(stair_cfg) = controller.stair_config.as_mut() {
            stair_settings_ui(ui, stair_cfg);
        }
    });
}
