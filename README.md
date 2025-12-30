# msg_character_controller

[![Crates.io](https://img.shields.io/crates/v/msg_character_controller.svg)](https://crates.io/crates/msg_character_controller)
[![Documentation](https://docs.rs/msg_character_controller/badge.svg)](https://docs.rs/msg_character_controller)
[![License](https://img.shields.io/crates/l/msg_character_controller.svg)](LICENSE-MIT)

An advanced 2D **floating rigidbody character controller** for the [Bevy](https://bevyengine.org) game engine with physics backend abstraction.

## Features

- **Floating rigidbody** - Hovers above ground using a spring-damper system rather than resting directly on surfaces
- **Physics backend abstraction** - Currently supports [Rapier2D](https://rapier.rs/) via `bevy_rapier2d`
- **Movement system** - Walking with slope handling, flying/jetpack propulsion
- **Jumping** - With coyote time, input buffering, and variable height (hold to jump higher)
- **Wall mechanics** - Wall jumping and wall clinging with configurable dampening
- **Stair stepping** - Automatic step climbing with shapecast detection
- **Gravity-relative directions** - Supports non-standard gravity (e.g., spherical planets)

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
msg_character_controller = "0.1"
bevy_rapier2d = "0.31"
```

The `rapier2d` feature is enabled by default.

## Quick Start

```rust
use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use msg_character_controller::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default())
        .add_systems(Startup, spawn_character)
        .add_systems(Update, handle_input)
        .run();
}

fn spawn_character(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, 100.0, 0.0),
        CharacterController::new(),
        ControllerConfig::player(),
        MovementIntent::default(),
        Rapier2dCharacterBundle::rotation_locked(),
        Collider::capsule_y(8.0, 4.0),
        GravityScale(0.0), // Controller manages gravity internally
    ));
}

fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut MovementIntent>,
) {
    for mut intent in &mut query {
        // Walking: A/D keys
        let walk = keyboard.pressed(KeyCode::KeyD) as i32
                 - keyboard.pressed(KeyCode::KeyA) as i32;
        intent.set_walk(walk as f32);

        // Jumping: Space key (controller handles buffering, coyote time, etc.)
        intent.set_jump_pressed(keyboard.pressed(KeyCode::Space));
    }
}
```

## Core Components

| Component | Purpose |
|-----------|---------|
| `CharacterController` | Central hub for state: collision data, timers, gravity |
| `ControllerConfig` | Tunable parameters: float height, speeds, jump settings |
| `MovementIntent` | Input abstraction: walk, fly, jump_pressed |
| `Rapier2dCharacterBundle` | Physics components for Rapier2D backend |

## Configuration

The controller is highly configurable through `ControllerConfig`:

```rust
let config = ControllerConfig::default()
    .with_float_height(6.0)          // Height to hover above ground
    .with_spring(300.0, 13.0)        // Spring strength and damping
    .with_max_speed(150.0)           // Maximum walk speed
    .with_jump_speed(120.0)          // Jump impulse strength
    .with_coyote_time(0.15)          // Time after leaving ground where jump still works
    .with_wall_jumping(true);        // Enable wall jumping
```

## Gravity

The controller manages gravity internally through `CharacterController::gravity`. Disable Rapier's gravity with `GravityScale(0.0)`:

```rust
// Standard downward gravity (default)
CharacterController::new()

// Custom gravity for spherical planets
CharacterController::with_gravity(Vec2::new(0.0, -500.0))
```

Use `set_gravity()` at runtime for dynamic gravity (e.g., spherical worlds):

```rust
fn update_gravity(mut query: Query<(&Transform, &mut CharacterController)>) {
    for (transform, mut controller) in &mut query {
        // Point toward planet center
        let to_center = -transform.translation.truncate();
        controller.set_gravity(to_center.normalize() * 980.0);
    }
}
```

## Examples

Run the examples with:

```bash
cargo run --example platform_box --features examples
cargo run --example spherical_planet --features examples
```

## Bevy Compatibility

| Bevy | msg_character_controller |
|------|--------------------------|
| 0.16 | 0.1                      |

## License

Dual-licensed under either:

* MIT License ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)
* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)

at your option.
