# msg_character_controller

Advanced 2D floating rigidbody character controller with physics backend abstraction for Bevy game engine.

## Features

- Floating rigidbody controller that hovers above ground
- Physics backend abstraction (currently supports Rapier2D)
- Configurable floating height, spring strength, and damping
- Ground detection and state management
- Automatic rigidbody configuration
- Movement intent system for AI and player control

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
msg_character_controller = { git = "https://github.com/ffmulks/msg_character_controller" }
```

To use with Rapier2D physics backend:

```toml
[dependencies]
msg_character_controller = { git = "https://github.com/ffmulks/msg_character_controller", features = ["rapier2d"] }
bevy_rapier2d = "0.31.0"
```

## Usage

```rust
use bevy::prelude::*;
use msg_character_controller::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(CharacterControllerPlugin)
        .run();
}

// Spawn a character with the controller
fn spawn_character(mut commands: Commands) {
    commands.spawn((
        CharacterControllerBundle::default(),
        // Add your other components here
    ));
}
```

## Configuration

The character controller can be configured through the `CharacterControllerConfig` component:

```rust
CharacterControllerConfig {
    float_height: 0.5,              // Height to float above ground
    float_spring_strength: 100.0,   // Spring strength for floating
    float_damping: 10.0,            // Damping for floating oscillations
    max_ground_distance: 1.0,       // Maximum distance to detect ground
    // ... other configuration options
}
```

## License

Dual-licensed under either:

* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)

at your option.