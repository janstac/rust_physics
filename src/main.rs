use notan::{
    app::{/*AppState,*/ WindowConfig},
    draw::{CreateDraw, DrawShapes, DrawTransform},
    egui,
    prelude::*,
};
mod physics_engine;
use physics_engine::*;
use std::time;

fn create_world() -> World {
    // Time in seconds that each physics step should be
    const STEP_TIME: f64 = 1.0 / 30.0;
    let mut world = World::new(STEP_TIME, 3, -1.0);
    world.bodies.push(
        BodyCreator {
            position: Vec2::new(0.0, -4.0),
            velocity: Vec2::new(0.0, 0.0),
            reciprocal_mass: 1.0,
            restitution: 1.0,
            angle: 0.0,
            angular_velocity: 0.0,
            shape: Shape::Circle { radius: 2.0 },
        }
        .build(),
    );
    world.bodies.push(
        BodyCreator {
            position: Vec2::new(-20.0, 0.0),
            velocity: Vec2::new(10.0, 0.0),
            reciprocal_mass: 1.0,
            restitution: 1.0,
            angle: 0.0,
            angular_velocity: 0.0,
            shape: Shape::Circle { radius: 2.0 },
        }
        .build(),
    );
    world.bodies.push(
        BodyCreator {
            position: Vec2::new(20.0, 0.0),
            velocity: Vec2::new(-10.0, 0.0),
            reciprocal_mass: 1.0,
            restitution: 1.0,
            angle: 0.0,
            angular_velocity: 0.0,
            shape: Shape::Circle { radius: 2.0 },
        }
        .build(),
    );
    world.bodies.push(
        BodyCreator {
            position: Vec2::new(-30.0, -20.0),
            velocity: Vec2::new(10.0, -3.0),
            reciprocal_mass: 1.0,
            restitution: 1.0,
            angle: 0.0,
            angular_velocity: 1.0,
            shape: Shape::Rectangle {
                size: Vec2::new(30.0, 4.0),
            },
        }
        .build(),
    );
    world.bodies.push(
        BodyCreator {
            position: Vec2::new(30.0, -20.0),
            velocity: Vec2::new(-10.0, -2.0),
            reciprocal_mass: 1.0,
            restitution: 1.0,
            angle: 0.2,
            angular_velocity: 0.0,
            shape: Shape::Rectangle {
                size: Vec2::new(30.0, 4.0),
            },
        }
        .build(),
    );
    world
}

#[derive(AppState, Clone)]
struct State {
    world: World,
    time_of_last_step: Option<time::Instant>,
}

#[notan::notan_main]
fn main() -> Result<(), String> {
    let window_config = WindowConfig::default()
        .set_resizable(true)
        // .set_size(1280, 1024)
        .set_vsync(true);
    // .set_high_dpi(true);

    let state = State {
        world: create_world(),
        time_of_last_step: None,
    };
    notan::init_with(|| state)
        .add_config(window_config)
        .add_config(egui::EguiConfig)
        .add_config(notan::draw::DrawConfig)
        // .add_plugin(notan::extra::FpsLimit::new(10))
        .update(update)
        .draw(draw)
        .build()
}

fn update(_app: &mut notan::app::App, _plugins: &mut notan::app::Plugins, state: &mut State) {
    let time_of_last_step = state
        .time_of_last_step
        .get_or_insert_with(time::Instant::now);
    let elapsed_duration = time_of_last_step.elapsed().as_secs_f64();
    let step_duration = state.world.step_time;
    let steps = (elapsed_duration / step_duration) as u8;
    for _ in 0..steps {
        state.world.step();
    }
    *time_of_last_step += time::Duration::from_secs_f64(step_duration * steps as f64);
}

struct CoordinateConverter {
    window_centre_x: f32,
    window_centre_y: f32,
    scale_factor: f64,
}
impl CoordinateConverter {
    fn x(&self, value: f64) -> f32 {
        self.window_centre_x + self.length(value)
    }
    fn y(&self, value: f64) -> f32 {
        self.window_centre_y - self.length(value)
    }
    fn length(&self, value: f64) -> f32 {
        (value * self.scale_factor) as f32
    }
}

fn draw(
    app: &mut notan::app::App,
    graphics: &mut notan::app::Graphics,
    _plugins: &mut notan::app::Plugins,
    state: &mut State,
) {
    let window = app.window();
    let coord = CoordinateConverter {
        window_centre_x: window.width() as f32 / 2.0,
        window_centre_y: window.height() as f32 / 2.0,
        scale_factor: 10.0,
    };
    let mut draw = graphics.create_draw();
    draw.clear(Color::GRAY);

    for body in state.world.bodies.iter() {
        match body.shape {
            Shape::Circle { radius } => {
                draw.circle(coord.length(radius))
                    .position(coord.x(body.p.position.x), coord.y(body.p.position.y))
                    .fill_color(Color::WHITE)
                    .fill();
            }
            Shape::Rectangle { size } => {
                let x = coord.x(body.p.position.x);
                let y = coord.y(body.p.position.y);
                let width = coord.length(size.x);
                let height = coord.length(size.y);
                draw.rect(
                    (coord.length(-size.x / 2.0), coord.length(-size.y / 2.0)),
                    (width, height),
                )
                .rotate(-body.p.angle as f32)
                .translate(x, y)
                .fill_color(Color::WHITE)
                .fill();
            }
        }
    }

    graphics.render(&draw);
}
