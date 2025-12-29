//! Plugin for character controller debug UI panels.
//!
//! Provides a unified Bevy plugin that automatically adds configuration and diagnostics
//! panels for character controllers using egui. This is the central UI solution for
//! all character controller examples.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use bevy_rapier2d::prelude::{ExternalForce, ExternalImpulse, Velocity};
use msg_character_controller::prelude::*;
use std::marker::PhantomData;

use super::{config_panel_ui, diagnostics_panel_ui, respawn_player, DiagnosticsData};

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

/// Resource storing panel configuration (positions, which panels to show).
#[derive(Resource)]
pub struct CharacterControllerUiPanelConfig {
    /// Whether to show the config panel.
    pub show_config_panel: bool,
    /// Whether to show the diagnostics panel.
    pub show_diagnostics_panel: bool,
    /// Default position for the config window.
    pub config_window_pos: [f32; 2],
    /// Default position for the diagnostics window.
    pub diagnostics_window_pos: [f32; 2],
}

impl Default for CharacterControllerUiPanelConfig {
    fn default() -> Self {
        Self {
            show_config_panel: true,
            show_diagnostics_panel: true,
            config_window_pos: [10.0, 80.0],
            diagnostics_window_pos: [320.0, 80.0],
        }
    }
}

/// Resource for spawn configuration (used by Respawn Player button).
///
/// Examples should insert this resource with their desired spawn position.
#[derive(Resource, Clone)]
pub struct SpawnConfig {
    /// The 2D position where the player should spawn.
    pub position: Vec2,
    /// Optional custom spawn logic that computes the spawn position dynamically.
    /// If Some, this takes precedence over the static `position` field.
    spawn_position_fn: Option<fn() -> Vec2>,
}

impl Default for SpawnConfig {
    fn default() -> Self {
        Self {
            position: Vec2::new(0.0, 100.0),
            spawn_position_fn: None,
        }
    }
}

impl SpawnConfig {
    /// Create a new spawn config with the given position.
    pub fn new(position: Vec2) -> Self {
        Self {
            position,
            spawn_position_fn: None,
        }
    }

    /// Create a spawn config with a dynamic position function.
    pub fn with_dynamic_position(position_fn: fn() -> Vec2) -> Self {
        Self {
            position: Vec2::ZERO,
            spawn_position_fn: Some(position_fn),
        }
    }

    /// Get the spawn position (either dynamic or static).
    pub fn get_position(&self) -> Vec2 {
        if let Some(f) = self.spawn_position_fn {
            f()
        } else {
            self.position
        }
    }
}

/// Resource for default controller configuration (used by Reset to Defaults button).
///
/// Examples should insert this resource with their desired default configuration.
#[derive(Resource, Clone)]
pub struct DefaultControllerSettings {
    /// The default controller config to restore.
    pub config: ControllerConfig,
    /// The default gravity vector to restore.
    pub gravity: Vec2,
}

impl Default for DefaultControllerSettings {
    fn default() -> Self {
        Self {
            config: ControllerConfig::player(),
            gravity: Vec2::new(0.0, -980.0),
        }
    }
}

impl DefaultControllerSettings {
    /// Create new default settings with the given config and gravity.
    pub fn new(config: ControllerConfig, gravity: Vec2) -> Self {
        Self { config, gravity }
    }
}

/// Resource that holds custom extra UI rendering logic.
///
/// This allows examples to add their own settings sections (like Planet Gravity)
/// to the unified settings panel without needing a separate type parameter.
#[derive(Resource, Default)]
pub struct ExtraSettingsUi {
    /// The extra UI rendering function.
    /// Called with a mutable reference to the egui Ui.
    render_fn: Option<fn(&mut egui::Ui, &mut World)>,
}

impl ExtraSettingsUi {
    /// Create with a custom rendering function.
    ///
    /// Note: The function receives the World for access to any resources needed.
    pub fn new(render_fn: fn(&mut egui::Ui, &mut World)) -> Self {
        Self {
            render_fn: Some(render_fn),
        }
    }

    /// Check if there's a render function.
    pub fn has_render_fn(&self) -> bool {
        self.render_fn.is_some()
    }
}

// ==================== Plugin Configuration ====================

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

// ==================== Plugin ====================

/// Unified plugin that adds character controller configuration and diagnostics UI panels.
///
/// This plugin provides:
/// - Settings panel with all controller configuration options
/// - Diagnostics panel showing real-time state
/// - Reset to Defaults button
/// - Respawn Player button
/// - Support for custom extra settings sections
///
/// # Type Parameters
///
/// - `M`: Marker component to identify which entity to show in the UI (e.g., `Player`)
///
/// # Customization
///
/// Examples can customize the plugin behavior by inserting these resources BEFORE adding the plugin:
///
/// - `SpawnConfig`: Set the spawn position for the Respawn Player button
/// - `DefaultControllerSettings`: Set the default config for Reset to Defaults button
///
/// For extra settings UI sections (like Planet Gravity), add the `extra_settings_ui` system
/// to `EguiPrimaryContextPass` after adding the plugin.
///
/// # Example
///
/// ```ignore
/// #[derive(Component)]
/// struct Player;
///
/// // Basic usage
/// App::new()
///     .add_plugins(CharacterControllerUiPlugin::<Player>::default())
///     // ...
///
/// // With custom spawn position and defaults
/// App::new()
///     .insert_resource(SpawnConfig::new(Vec2::new(0.0, 200.0)))
///     .insert_resource(DefaultControllerSettings::new(
///         ControllerConfig::player().with_float_height(5.0),
///         Vec2::new(0.0, -980.0),
///     ))
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

        // Insert panel config resource
        app.insert_resource(CharacterControllerUiPanelConfig {
            show_config_panel: self.config.show_config_panel,
            show_diagnostics_panel: self.config.show_diagnostics_panel,
            config_window_pos: self.config.config_window_pos,
            diagnostics_window_pos: self.config.diagnostics_window_pos,
        });

        // Initialize default resources if not already present
        app.init_resource::<SpawnConfig>();
        app.init_resource::<DefaultControllerSettings>();

        // Add the UI systems
        app.add_systems(
            EguiPrimaryContextPass,
            (
                toggle_panels_visibility,
                show_help_text,
                character_controller_settings_ui_system::<M>,
                character_controller_diagnostics_ui_system::<M>,
            )
                .chain(),
        );
    }
}

// ==================== Systems ====================

/// System to toggle panel visibility.
fn toggle_panels_visibility(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut ui_state: ResMut<CharacterControllerUiState>,
) {
    // Increment frame counter
    ui_state.frame_count += 1;

    // Toggle panels visibility
    if keyboard.just_pressed(ui_state.toggle_key) {
        ui_state.show_panels = !ui_state.show_panels;
    }
}

/// System to show help text.
fn show_help_text(mut contexts: EguiContexts, ui_state: Res<CharacterControllerUiState>) {
    // Skip the first few frames to ensure egui is fully initialized
    if ui_state.frame_count <= 2 {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

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
}

/// System that renders the settings panel.
fn character_controller_settings_ui_system<M: Component>(
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
        ),
        With<M>,
    >,
    ui_state: Res<CharacterControllerUiState>,
    panel_config: Res<CharacterControllerUiPanelConfig>,
    spawn_config: Res<SpawnConfig>,
    default_settings: Res<DefaultControllerSettings>,
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

    // Settings panel
    if !panel_config.show_config_panel {
        return;
    }

    let Ok((
        mut config,
        mut controller,
        mut transform,
        mut velocity,
        mut external_impulse,
        mut external_force,
        mut movement_intent,
    )) = config_query.single_mut()
    else {
        return;
    };

    egui::Window::new("Controller Settings")
        .default_pos(panel_config.config_window_pos)
        .default_width(300.0)
        .default_height(400.0)
        .collapsible(true)
        .resizable(true)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                // Reset and Respawn buttons at the top
                ui.horizontal(|ui| {
                    if ui.button("Reset to Defaults").clicked() {
                        *config = default_settings.config.clone();
                        controller.gravity = default_settings.gravity;
                    }
                    if ui.button("Respawn Player").clicked() {
                        respawn_player(
                            spawn_config.get_position(),
                            &mut transform,
                            &mut velocity,
                            &mut external_impulse,
                            &mut external_force,
                            &mut controller,
                            &mut movement_intent,
                        );
                    }
                });
                ui.add_space(8.0);

                // Standard config panel
                config_panel_ui(ui, &mut config, &mut controller);
            });
        });
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
    if !panel_config.show_diagnostics_panel {
        return;
    }

    let Ok((
        config_ref,
        controller_ref,
        transform_ref,
        velocity_ref,
        movement,
        ext_force,
        ext_impulse,
    )) = diagnostics_query.single()
    else {
        return;
    };

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
