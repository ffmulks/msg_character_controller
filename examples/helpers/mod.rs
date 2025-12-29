//! Helper modules for examples.
//!
//! This module provides reusable UI panels and plugins for egui-based examples.

// Allow unused items since different examples use different subsets of helpers
#![allow(dead_code)]

mod config_panel;
mod controls_plugin;
mod diagnostics_panel;
mod mesh_shapes;
mod plugin;
mod respawn;

#[allow(unused_imports)]
pub use config_panel::*;
pub use controls_plugin::*;
#[allow(unused_imports)]
pub use diagnostics_panel::*;
#[allow(unused_imports)]
pub use mesh_shapes::*;
#[allow(unused_imports)]
pub use plugin::*;
#[allow(unused_imports)]
pub use respawn::*;
