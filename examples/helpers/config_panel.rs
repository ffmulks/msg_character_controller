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
}

/// Renders the spring settings collapsible section.
pub fn spring_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
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
}

/// Renders the movement settings collapsible section.
pub fn movement_settings_ui(ui: &mut egui::Ui, config: &mut ControllerConfig) {
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
                    .range(0.0..=5.0),
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
        upright_torque_settings_ui(ui, config);
    });
}
