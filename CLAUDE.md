# Project Notes for Claude

## Overview

`msg_character_controller` is an advanced 2D **floating rigidbody character controller** for the Bevy game engine with physics backend abstraction. The controller hovers above the ground using a spring-damper system rather than resting directly on surfaces.

**Key Features:**
- Floating rigidbody that hovers above ground
- Spring-damper system for ground detection and height maintenance
- Physics backend abstraction (currently supports Rapier2D via `bevy_rapier2d`)
- Movement intent system for player/AI control
- Jump with coyote time and input buffering
- Wall jumping and wall clinging
- Stair stepping
- Slope handling
- Gravity-relative directions (supports non-standard gravity like spherical planets)

## Rapier Physics Integration

**IMPORTANT**: Rapier runs in `PostFixedUpdate`, NOT in `FixedUpdate`.

Do NOT order CharacterControllerSet relative to Rapier's PhysicsSet. The character controller systems run in FixedUpdate, and Rapier processes ExternalForce/ExternalImpulse in PostFixedUpdate after FixedUpdate completes.

The correct flow is:
1. FixedUpdate: Character controller systems accumulate forces into ExternalForce/ExternalImpulse
2. PostFixedUpdate: Rapier reads ExternalForce/ExternalImpulse and simulates physics

## Architecture

### Core Components

| Component | Purpose |
|-----------|---------|
| `CharacterController` | Central hub for state: collision data, timers, gravity, force accumulation |
| `ControllerConfig` | Tunable parameters: float height, spring strength, speeds, jump settings |
| `MovementIntent` | Input abstraction: walk, fly, jump_pressed |
| `StairConfig` | Optional stair stepping configuration |
| `CollisionData` | Stores raycast hit info (distance, normal, point, entity) |

### Physics Backend Trait

`CharacterPhysicsBackend` (`src/backend.rs`) abstracts physics engine operations:
- `get_velocity`, `set_velocity`, `apply_impulse`, `apply_force`
- `get_mass`, `get_principal_inertia`
- `get_rotation`, `get_angular_velocity`, `apply_torque`

The Rapier2D implementation is in `src/rapier.rs` behind the `rapier2d` feature.

### System Phases

Systems run in `FixedUpdate` in six ordered phases via `CharacterControllerSet`:

1. **Preparation** - Process jump state (edge detection), tick timers, remove expired jump requests
2. **Sensors** - Ground/wall/ceiling detection via raycasts (runs in Rapier backend plugin)
3. **IntentEvaluation** - Update timers, determine jump type, evaluate intent flags
4. **ForceAccumulation** - Accumulate spring force, stair climb force, gravity, upright torque
5. **IntentApplication** - Apply fall gravity, jump impulse, walk impulse, fly impulse, wall clinging
6. **FinalApplication** - Apply accumulated forces to physics (runs in Rapier backend plugin)

## Key Files

| File | Description |
|------|-------------|
| `src/lib.rs` | Plugin definition, system set ordering, system registration |
| `src/config.rs` | `CharacterController`, `ControllerConfig`, `StairConfig`, `JumpType` |
| `src/intent.rs` | `MovementIntent`, `JumpRequest` |
| `src/systems.rs` | All controller systems (spring, gravity, movement, jump, etc.) |
| `src/backend.rs` | `CharacterPhysicsBackend` trait |
| `src/rapier.rs` | Rapier2D backend implementation, sensor systems |
| `src/collision.rs` | `CollisionData` struct |

## Building & Running

```bash
# Run tests
cargo test

# Run specific example
cargo run --example platform_box --features examples
cargo run --example spherical_planet --features examples

# Build documentation
cargo doc --open
```

## Testing

Integration tests are in `tests/integration.rs`. Tests use a minimal Bevy app with Rapier physics:

```rust
fn create_test_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
    app.add_plugins(CharacterControllerPlugin::<Rapier2dBackend>::default());
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.finish();
    app.cleanup();
    app
}
```

Characters are spawned with `GravityScale(0.0)` to disable Rapier's gravity (controller manages gravity internally via `CharacterController::gravity`).

## Common Patterns

### Spawning a Character

```rust
commands.spawn((
    Transform::from_translation(position.extend(0.0)),
    CharacterController::new(),  // or ::with_gravity(custom_gravity)
    ControllerConfig::default(), // or ::player() / ::ai()
    MovementIntent::default(),
    Rapier2dCharacterBundle::rotation_locked(), // or ::default() for rotation
    Collider::capsule_y(8.0, 4.0),
    GravityScale(0.0), // Disable Rapier gravity
));
```

### Handling Input

```rust
fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut MovementIntent>,
) {
    for mut intent in &mut query {
        // Walking
        let walk = keyboard.pressed(KeyCode::KeyD) as i32 - keyboard.pressed(KeyCode::KeyA) as i32;
        intent.set_walk(walk as f32);

        // Jumping (controller handles edge detection, buffering, coyote time)
        intent.set_jump_pressed(keyboard.pressed(KeyCode::Space));
    }
}
```

### Ideal Directions

The controller uses gravity-relative directions via `CharacterController`:
- `ideal_up()` - opposite of gravity direction
- `ideal_down()` - same as gravity direction
- `ideal_right()` / `ideal_left()` - perpendicular to gravity

This enables spherical planet gameplay where "up" varies by position.

## Important Implementation Notes

1. **Force Isolation**: The controller tracks forces it applies to avoid interfering with external forces. Forces are subtracted at frame start and applied at frame end.

2. **Jump Spring Filtering**: After jumping or flying up, downward spring forces are filtered for `jump_spring_filter_duration` to prevent counteracting the upward motion.

3. **Coyote Time**: Tracks time since last grounded/wall contact. Jump requests are valid during this window even when airborne.

4. **Jump Buffering**: Jump requests have a timer. If contact is made before the timer expires, the jump executes.

5. **Wall Jump Movement Blocking**: After a wall jump, movement toward the wall is temporarily blocked to help the player jump away correctly.

6. **Recently Jumped Protection**: A brief window after jumping prevents fall gravity from triggering due to velocity flicker and prevents coyote timer reset.

## Edition

This project uses Rust edition 2024.
