//! Diagnostics panel for CharacterController state.
//!
//! Provides a read-only egui panel displaying the current state of
//! a character controller for debugging and visualization.

use bevy::prelude::*;
use bevy_egui::egui;
use bevy_rapier2d::prelude::Velocity;
use msg_character_controller::prelude::*;

/// Data needed for the diagnostics panel.
pub struct DiagnosticsData<'a> {
    pub controller: &'a CharacterController,
    pub config: &'a ControllerConfig,
    pub transform: &'a Transform,
    pub velocity: &'a Velocity,
    pub movement_intent: Option<&'a MovementIntent>,
    pub grounded: bool,
    pub touching_wall: Option<&'a TouchingWall>,
    pub touching_ceiling: Option<&'a TouchingCeiling>,
}

/// Renders the position and velocity section.
pub fn position_velocity_ui(ui: &mut egui::Ui, transform: &Transform, velocity: &Velocity) {
    ui.collapsing("Position & Velocity", |ui| {
        ui.horizontal(|ui| {
            ui.label("Position:");
            ui.label(format!(
                "({:.1}, {:.1})",
                transform.translation.x, transform.translation.y
            ));
        });
        ui.horizontal(|ui| {
            ui.label("Rotation:");
            ui.label(format!(
                "{:.1} deg",
                transform.rotation.to_euler(EulerRot::ZYX).0.to_degrees()
            ));
        });
        ui.horizontal(|ui| {
            ui.label("Velocity:");
            ui.label(format!(
                "({:.1}, {:.1})",
                velocity.linvel.x, velocity.linvel.y
            ));
        });
        ui.horizontal(|ui| {
            ui.label("Speed:");
            ui.label(format!("{:.1}", velocity.linvel.length()));
        });
        ui.horizontal(|ui| {
            ui.label("Angular Vel:");
            ui.label(format!("{:.2} rad/s", velocity.angvel));
        });
    });
}

/// Renders the ground detection section.
pub fn ground_detection_ui(
    ui: &mut egui::Ui,
    controller: &CharacterController,
    config: &ControllerConfig,
) {
    let grounded = controller.is_grounded(config);
    ui.collapsing("Ground Detection", |ui| {
        let state_color = if grounded {
            egui::Color32::from_rgb(100, 200, 100)
        } else {
            egui::Color32::from_rgb(200, 100, 100)
        };
        ui.horizontal(|ui| {
            ui.label("State:");
            ui.colored_label(state_color, if grounded { "GROUNDED" } else { "AIRBORNE" });
        });

        ui.horizontal(|ui| {
            ui.label("Floor Detected:");
            ui.label(if controller.floor.is_some() {
                "Yes"
            } else {
                "No"
            });
        });

        if let Some(ref floor) = controller.floor {
            ui.horizontal(|ui| {
                ui.label("Floor Distance:");
                ui.label(format!("{:.2}", floor.distance));
            });
            ui.horizontal(|ui| {
                ui.label("Floor Normal:");
                ui.label(format!("({:.2}, {:.2})", floor.normal.x, floor.normal.y));
            });
            ui.horizontal(|ui| {
                ui.label("Contact Point:");
                ui.label(format!("({:.1}, {:.1})", floor.point.x, floor.point.y));
            });
        }

        ui.horizontal(|ui| {
            ui.label("Riding Height:");
            ui.label(format!("{:.2}", controller.riding_height(config)));
        });

        ui.horizontal(|ui| {
            ui.label("In Spring Range:");
            let in_range = controller.in_spring_range(config);
            let color = if in_range {
                egui::Color32::from_rgb(100, 200, 100)
            } else {
                egui::Color32::from_rgb(150, 150, 150)
            };
            ui.colored_label(color, if in_range { "Yes" } else { "No" });
        });

        ui.horizontal(|ui| {
            ui.label("Slope Angle:");
            ui.label(format!("{:.1} deg", controller.slope_angle.to_degrees()));
        });

        ui.horizontal(|ui| {
            ui.label("Time Since Grounded:");
            ui.label(format!("{:.3}s", controller.time_since_grounded));
        });
    });
}

/// Renders the wall and ceiling detection section.
pub fn wall_ceiling_ui(ui: &mut egui::Ui, controller: &CharacterController) {
    ui.collapsing("Wall & Ceiling", |ui| {
        // Left wall
        ui.horizontal(|ui| {
            ui.label("Left Wall:");
            if let Some(ref wall) = controller.left_wall {
                ui.colored_label(
                    egui::Color32::from_rgb(200, 150, 100),
                    format!("dist={:.2}, normal=({:.2}, {:.2})", wall.distance, wall.normal.x, wall.normal.y),
                );
            } else {
                ui.label("None");
            }
        });

        // Right wall
        ui.horizontal(|ui| {
            ui.label("Right Wall:");
            if let Some(ref wall) = controller.right_wall {
                ui.colored_label(
                    egui::Color32::from_rgb(200, 150, 100),
                    format!("dist={:.2}, normal=({:.2}, {:.2})", wall.distance, wall.normal.x, wall.normal.y),
                );
            } else {
                ui.label("None");
            }
        });

        // Ceiling
        ui.horizontal(|ui| {
            ui.label("Ceiling:");
            if let Some(ref ceiling) = controller.ceiling {
                ui.colored_label(
                    egui::Color32::from_rgb(200, 150, 100),
                    format!("dist={:.2}, normal=({:.2}, {:.2})", ceiling.distance, ceiling.normal.x, ceiling.normal.y),
                );
            } else {
                ui.label("None");
            }
        });

        // Summary
        ui.horizontal(|ui| {
            ui.label("Touching Wall:");
            let touching = controller.touching_wall();
            let color = if touching {
                egui::Color32::from_rgb(200, 150, 100)
            } else {
                egui::Color32::from_rgb(150, 150, 150)
            };
            ui.colored_label(color, if touching { "Yes" } else { "No" });
        });

        ui.horizontal(|ui| {
            ui.label("Touching Ceiling:");
            let touching = controller.touching_ceiling();
            let color = if touching {
                egui::Color32::from_rgb(200, 150, 100)
            } else {
                egui::Color32::from_rgb(150, 150, 150)
            };
            ui.colored_label(color, if touching { "Yes" } else { "No" });
        });
    });
}

/// Renders the movement intent section.
pub fn movement_intent_ui(ui: &mut egui::Ui, movement: Option<&MovementIntent>) {
    ui.collapsing("Movement Intent", |ui| {
        if let Some(intent) = movement {
            ui.horizontal(|ui| {
                ui.label("Walk:");
                let walk_str = if intent.walk > 0.01 {
                    format!("Right ({:.2})", intent.walk)
                } else if intent.walk < -0.01 {
                    format!("Left ({:.2})", intent.walk)
                } else {
                    "None".to_string()
                };
                ui.label(walk_str);
            });
            ui.horizontal(|ui| {
                ui.label("Fly:");
                let fly_str = if intent.fly > 0.01 {
                    format!("Up ({:.2})", intent.fly)
                } else if intent.fly < -0.01 {
                    format!("Down ({:.2})", intent.fly)
                } else {
                    "None".to_string()
                };
                ui.label(fly_str);
            });
            ui.horizontal(|ui| {
                ui.label("Walk Speed:");
                ui.label(format!("{:.2}", intent.walk_speed));
            });
            ui.horizontal(|ui| {
                ui.label("Fly Speed:");
                ui.label(format!("{:.2}", intent.fly_speed));
            });

            ui.separator();

            ui.horizontal(|ui| {
                ui.label("Jump Requested:");
                let color = if intent.jump_request.is_some() {
                    egui::Color32::from_rgb(100, 200, 100)
                } else {
                    egui::Color32::from_rgb(150, 150, 150)
                };
                ui.colored_label(
                    color,
                    if intent.jump_request.is_some() {
                        "ACTIVE"
                    } else {
                        "No"
                    },
                );
            });
            if let Some(ref jump) = intent.jump_request {
                ui.horizontal(|ui| {
                    ui.label("Request Time:");
                    ui.label(format!("{:.3}s", jump.request_time));
                });
            }
        } else {
            ui.label("No MovementIntent component");
        }
    });
}

/// Renders the internal state section.
pub fn internal_state_ui(ui: &mut egui::Ui, controller: &CharacterController) {
    ui.collapsing("Internal State", |ui| {
        ui.horizontal(|ui| {
            ui.label("Gravity:");
            ui.label(format!(
                "({:.1}, {:.1})",
                controller.gravity.x, controller.gravity.y
            ));
        });
        ui.horizontal(|ui| {
            ui.label("Collider Bottom Offset:");
            ui.label(format!("{:.2}", controller.capsule_half_height()));
        });
        ui.horizontal(|ui| {
            ui.label("Step Detected:");
            ui.label(if controller.step_detected {
                format!("Yes (h={:.2})", controller.step_height)
            } else {
                "No".to_string()
            });
        });
    });
}

/// Renders the complete diagnostics panel with all sections.
///
/// This is a convenience function that renders all diagnostic sections
/// in a single scrollable area.
pub fn diagnostics_panel_ui(ui: &mut egui::Ui, data: &DiagnosticsData) {
    egui::ScrollArea::vertical().show(ui, |ui| {
        position_velocity_ui(ui, data.transform, data.velocity);
        ground_detection_ui(ui, data.controller, data.config, data.grounded);
        wall_ceiling_ui(ui, data.controller, data.touching_wall, data.touching_ceiling);
        movement_intent_ui(ui, data.movement_intent);
        internal_state_ui(ui, data.controller);
    });
}
