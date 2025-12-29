//! Helper modules for examples.
//!
//! This module provides reusable UI panels and plugins for egui-based examples.

mod config_panel;
mod controls_plugin;
mod diagnostics_panel;
mod plugin;
mod respawn;

pub use config_panel::*;
pub use controls_plugin::*;
pub use diagnostics_panel::*;
pub use plugin::*;
pub use respawn::*;
