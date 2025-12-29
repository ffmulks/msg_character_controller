//! Plugin for character controller debug UI panels.
//!
//! Provides a Bevy plugin that automatically adds configuration and diagnostics
//! panels for character controllers using egui.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use bevy_rapier2d::prelude::{ExternalForce, ExternalImpulse, Velocity};
use msg_character_controller::prelude::*;
use std::marker::PhantomData;

use super::{config_panel_ui, diagnostics_panel_ui, DiagnosticsData};

/// Resource containing the UI panel state.
#[derive(Resource)]
pub struct CharacterControllerUiState {
    /// Number of frames since startup (used to skip initial frames).
    pub frame_count: u32,
    /// Whether the panels are currently visible.
    pub show_panels: bool,
    /// Key to toggle panel visibility.
    pub toggle_key: KeyCode,
}

impl Default for CharacterControllerUiState {
    fn default() -> Self {
        Self {
            frame_count: 0,
            show_panels: true,
            toggle_key: KeyCode::Tab,
        }
    }
}

/// Configuration for the character controller UI plugin.
pub struct CharacterControllerUiConfig {
    /// Whether to show the config panel.
    pub show_config_panel: bool,
    /// Whether to show the diagnostics panel.
    pub show_diagnostics_panel: bool,
    /// Default position for the config window.
    pub config_window_pos: [f32; 2],
    /// Default position for the diagnostics window.
    pub diagnostics_window_pos: [f32; 2],
    /// Key to toggle panel visibility.
    pub toggle_key: KeyCode,
    /// Whether panels are visible at startup.
    pub visible_at_startup: bool,
}

impl Default for CharacterControllerUiConfig {
    fn default() -> Self {
        Self {
            show_config_panel: true,
            show_diagnostics_panel: true,
            config_window_pos: [10.0, 80.0],
            diagnostics_window_pos: [320.0, 80.0],
            toggle_key: KeyCode::Tab,
            visible_at_startup: true,
        }
    }
}

/// Plugin that adds character controller configuration and diagnostics UI panels.
///
/// This plugin requires a marker component type `M` to identify which entity
/// should be shown in the UI. Typically this is a `Player` component.
///
/// # Example
///
/// ```ignore
/// #[derive(Component)]
/// struct Player;
///
/// App::new()
///     .add_plugins(CharacterControllerUiPlugin::<Player>::default())
///     // ...
/// ```
pub struct CharacterControllerUiPlugin<M: Component> {
    config: CharacterControllerUiConfig,
    _marker: PhantomData<M>,
}

impl<M: Component> Default for CharacterControllerUiPlugin<M> {
    fn default() -> Self {
        Self {
            config: CharacterControllerUiConfig::default(),
            _marker: PhantomData,
        }
    }
}

impl<M: Component> CharacterControllerUiPlugin<M> {
    /// Create a new plugin with default configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set whether the config panel should be shown.
    pub fn with_config_panel(mut self, show: bool) -> Self {
        self.config.show_config_panel = show;
        self
    }

    /// Set whether the diagnostics panel should be shown.
    pub fn with_diagnostics_panel(mut self, show: bool) -> Self {
        self.config.show_diagnostics_panel = show;
        self
    }

    /// Set the default position for the config window.
    pub fn with_config_window_pos(mut self, pos: [f32; 2]) -> Self {
        self.config.config_window_pos = pos;
        self
    }

    /// Set the default position for the diagnostics window.
    pub fn with_diagnostics_window_pos(mut self, pos: [f32; 2]) -> Self {
        self.config.diagnostics_window_pos = pos;
        self
    }

    /// Set the key used to toggle panel visibility.
    pub fn with_toggle_key(mut self, key: KeyCode) -> Self {
        self.config.toggle_key = key;
        self
    }

    /// Set whether panels should be visible at startup.
    pub fn with_visible_at_startup(mut self, visible: bool) -> Self {
        self.config.visible_at_startup = visible;
        self
    }
}

impl<M: Component> Plugin for CharacterControllerUiPlugin<M> {
    fn build(&self, app: &mut App) {
        // Insert UI state resource
        app.insert_resource(CharacterControllerUiState {
            frame_count: 0,
            show_panels: self.config.visible_at_startup,
            toggle_key: self.config.toggle_key,
        });

        // Insert config resource
        app.insert_resource(CharacterControllerUiPanelConfig {
            show_config_panel: self.config.show_config_panel,
            show_diagnostics_panel: self.config.show_diagnostics_panel,
            config_window_pos: self.config.config_window_pos,
            diagnostics_window_pos: self.config.diagnostics_window_pos,
        });

        // Add the UI systems
        app.add_systems(
            EguiPrimaryContextPass,
            (
                character_controller_config_ui_system::<M>,
                character_controller_diagnostics_ui_system::<M>,
            ),
        );
    }
}

/// Resource storing panel configuration (positions, which panels to show).
#[derive(Resource)]
struct CharacterControllerUiPanelConfig {
    show_config_panel: bool,
    show_diagnostics_panel: bool,
    config_window_pos: [f32; 2],
    diagnostics_window_pos: [f32; 2],
}

/// System that renders the config panel and help text.
fn character_controller_config_ui_system<M: Component>(
    mut contexts: EguiContexts,
    mut config_query: Query<
        (&mut ControllerConfig, &mut CharacterController),
        With<M>,
    >,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut ui_state: ResMut<CharacterControllerUiState>,
    panel_config: Res<CharacterControllerUiPanelConfig>,
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

    // Toggle panels visibility
    if keyboard.just_pressed(ui_state.toggle_key) {
        ui_state.show_panels = !ui_state.show_panels;
    }

    // Show help text
    egui::Area::new(egui::Id::new("cc_ui_info_area"))
        .fixed_pos(egui::pos2(10.0, 40.0))
        .show(ctx, |ui| {
            ui.colored_label(
                egui::Color32::from_rgb(200, 200, 200),
                if ui_state.show_panels {
                    format!("Press {:?} to hide panels", ui_state.toggle_key)
                } else {
                    format!("Press {:?} to show panels", ui_state.toggle_key)
                },
            );
        });

    if !ui_state.show_panels {
        return;
    }

    // Config panel
    if panel_config.show_config_panel {
        if let Ok((mut config, mut controller)) = config_query.single_mut() {
            egui::Window::new("Controller Settings")
                .default_pos(panel_config.config_window_pos)
                .default_width(300.0)
                .default_height(400.0)
                .collapsible(true)
                .resizable(true)
                .show(ctx, |ui| {
                    config_panel_ui(ui, &mut config, &mut controller);
                });
        }
    }
}

/// System that renders the diagnostics panel.
fn character_controller_diagnostics_ui_system<M: Component>(
    mut contexts: EguiContexts,
    diagnostics_query: Query<
        (
            &ControllerConfig,
            &CharacterController,
            &Transform,
            &Velocity,
            Option<&MovementIntent>,
            Option<&ExternalForce>,
            Option<&ExternalImpulse>,
        ),
        With<M>,
    >,
    ui_state: Res<CharacterControllerUiState>,
    panel_config: Res<CharacterControllerUiPanelConfig>,
) {
    // Skip the first few frames to ensure egui is fully initialized
    if ui_state.frame_count <= 2 {
        return;
    }

    if !ui_state.show_panels {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    // Diagnostics panel
    if panel_config.show_diagnostics_panel {
        if let Ok((
            config_ref,
            controller_ref,
            transform_ref,
            velocity_ref,
            movement,
            ext_force,
            ext_impulse,
        )) = diagnostics_query.single()
        {
            egui::Window::new("Diagnostics")
                .default_pos(panel_config.diagnostics_window_pos)
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
                        external_force: ext_force,
                        external_impulse: ext_impulse,
                    };
                    diagnostics_panel_ui(ui, &data);
                });
        }
    }
}
