mod collision_detection;
pub mod vector;

pub use vector::{cross_scalar_vec, cross_vec_vec, Float, Vec2};

#[derive(Clone)]
pub struct BodyProperties {
    pub position: Vec2,
    pub previous_position: Vec2,
    pub velocity: Vec2,
    pub previous_velocity: Vec2,
    pub acceleration: Vec2,
    pub inverse_mass: Float,
    pub restitution: Float,
    pub friction_coefficient: Float,
    pub angle: Float,
    pub angular_velocity: Float,
    pub inverse_inertia: Float,
}

#[derive(Clone)]
pub struct Body {
    /// Properties
    pub p: BodyProperties,
    pub shape: Shape,
}

#[derive(Clone)]
pub struct BodyCreator {
    /// Position of centre of shape
    pub position: Vec2,
    pub velocity: Vec2,
    pub acceleration: Vec2,
    /// 1 divided by mass, can be 0
    pub inverse_mass: Float,
    pub restitution: Float,
    pub friction_coefficient: Float,

    /// Anticlockwise angle in radians around centre
    pub angle: Float,
    pub angular_velocity: Float,

    pub shape: Shape,
}

impl BodyCreator {
    pub fn build(self) -> Body {
        Body {
            p: BodyProperties {
                position: self.position,
                previous_position: self.position,
                velocity: self.velocity,
                previous_velocity: self.velocity,
                acceleration: self.acceleration,

                inverse_mass: self.inverse_mass,
                restitution: self.restitution,
                friction_coefficient: self.friction_coefficient,
                angle: self.angle,
                angular_velocity: self.angular_velocity,
                inverse_inertia: calculate_inverse_inertia(self.inverse_mass, &self.shape)
            },
            shape: self.shape,
        }
    }
}

#[derive(Clone)]
pub enum Shape {
    Circle {
        radius: Float,
    },
    /// Rectangle centred at body position
    Rectangle {
        /// Vector that contains width and height
        size: Vec2,
    },
}

#[derive(Clone)]
pub struct World {
    pub bodies: Vec<Body>,
    pub step_time: Float,
    pub substeps: u8,
    pub contacts: Vec<Vec2>, // TODO temp
}

struct Collision {
    body1_index: usize,
    body2_index: usize,
    contact: CollisionContact,
}

#[derive(Clone, Debug)]
struct CollisionContact {
    /// Unit vector from body 1 to 2 that is perpendicular to the collision edge
    normal: Vec2,
    intersection_depth: Float,
    contact_point: Vec2,
}

impl World {
    pub fn new(delta_time: Float, substeps: u8) -> World {
        World {
            bodies: vec![],
            step_time: delta_time,
            substeps,
            contacts: vec![], // TODO temp
        }
    }
    pub fn step(&mut self) {
        // At the moment, this just creates a list of every possible pairing of bodies
        // TODO: eliminate pairs somehow
        let possible_collision_pair_indexes = {
            let mut result = Vec::new();
            for i in 0..self.bodies.len() {
                for j in i + 1..self.bodies.len() {
                    result.push((i, j));
                }
            }
            result
        };

        let substep_time: Float = self.step_time / self.substeps as Float;
        for _ in 0..self.substeps {
            for body in self.bodies.iter_mut() {
                body.p.previous_position = body.p.position;
                body.p.previous_velocity = body.p.velocity;
                body.p.velocity += body.p.acceleration * substep_time;

                // TODO
                // If the body is being accelerated, there is a more accurate way to calculate the new position
                body.p.position += body.p.velocity * substep_time;
                body.p.angle += body.p.angular_velocity * substep_time;
            }

            let collisions = find_collisions(
                self.bodies.as_mut_slice(),
                possible_collision_pair_indexes.as_slice(),
            );

            // for collision in collisions.iter() {
            //     self.contacts.push(collision.contact.contact_point); // TODO temp
            // }

            resolve_collisions(self.bodies.as_mut_slice(), collisions.as_slice());
        }
    }
}

// Compute inverse moment of inertia from shape and inverse mass
fn calculate_inverse_inertia(inv_m: Float, shape: &Shape) -> Float {
    if inv_m == 0.0 {
        return 0.0;
    }
    match shape {
        &Shape::Circle { radius } => {
            // I = 0.5 * m * r^2  => invI = 2 * inv_m / r^2
            if radius <= 0.0 {
                0.0
            } else {
                2.0 * inv_m / (radius * radius)
            }
        }
        &Shape::Rectangle { size } => {
            // I = (1/12) * m * (w^2 + h^2) => invI = 12 * inv_m / (w^2 + h^2)
            let w2h2 = size.x * size.x + size.y * size.y;
            if w2h2 <= 0.0 {
                0.0
            } else {
                12.0 * inv_m / w2h2
            }
        }
    }
}

fn resolve_collisions(bodies: &mut [Body], collisions: &[Collision]) {
    for &Collision {
        body1_index,
        body2_index,
        ref contact,
    } in collisions
    {
        let (body1, body2) = body_pair_mut(bodies, body1_index, body2_index);

        // Move bodies by amounts proportional to their inverse masses so they no longer intersect

        // inverse_masses_sum==0 has alredy been checked
        let inverse_masses_sum = body1.p.inverse_mass + body2.p.inverse_mass;

        let body1_displacement_size =
            contact.intersection_depth * body1.p.inverse_mass / inverse_masses_sum;
        let body2_displacement_size =
            contact.intersection_depth * body2.p.inverse_mass / inverse_masses_sum;

        // The contact normal points from body 1 to 2, so -= for 1 and += for 2
        body1.p.position -= contact.normal * body1_displacement_size;
        body2.p.position += contact.normal * body2_displacement_size;

        // Impulse resolution including angular effects

        // Contact point and radii from centres
        let contact_point = contact.contact_point;
        let r1 = contact_point - body1.p.position;
        let r2 = contact_point - body2.p.position;

        // Relative velocity at contact
        // r x angular_velocity is velocity due to rotation
        let vel1_at_contact = body1.p.velocity + cross_scalar_vec(body1.p.angular_velocity, r1);
        let vel2_at_contact = body2.p.velocity + cross_scalar_vec(body2.p.angular_velocity, r2);

        // >0 means separating
        let relative_velocity_along_normal =
            (vel2_at_contact - vel1_at_contact).dot(&contact.normal);

        // Only resolve if bodies are approaching along normal
        if relative_velocity_along_normal > 0.0 {
            continue;
        }

        let bounce_threshold = 0.3;
        let restitution = if -relative_velocity_along_normal < bounce_threshold {
            0.0
        } else {
            // Average restitution
            (body1.p.restitution + body2.p.restitution) / 2.0
        };

        // Rotational contribution: (r × n)^2 * invI
        let r1n = cross_vec_vec(r1, contact.normal);
        let r2n = cross_vec_vec(r2, contact.normal);
        let inv_inertia_sum = r1n * r1n * body1.p.inverse_inertia + r2n * r2n * body2.p.inverse_inertia;

        let denom = inverse_masses_sum + inv_inertia_sum;
        if denom <= 0.0 {
            continue;
        }
        let impulse_scalar = -(1.0 + restitution) * relative_velocity_along_normal / denom;
        let impulse = contact.normal * impulse_scalar;

        // Apply linear impulses
        body1.p.velocity -= impulse * body1.p.inverse_mass;
        body2.p.velocity += impulse * body2.p.inverse_mass;

        // Apply angular impulses (Δω = invI * τ, τ = r × F; body1 gets -impulse, body2 gets +impulse)
        let tau1 = cross_vec_vec(r1, impulse);
        let tau2 = cross_vec_vec(r2, impulse);
        body1.p.angular_velocity -= body1.p.inverse_inertia * tau1;
        body2.p.angular_velocity += body2.p.inverse_inertia * tau2;

        // Friction

        let tangent = {
            let n = contact.normal;
            Vec2::new(-n.y, n.x)
        };
        let r1tangent = cross_vec_vec(r1, tangent);
        let r2tangent = cross_vec_vec(r2, tangent);

        let denom_tangent = inverse_masses_sum
            + r1tangent * r1tangent * body1.p.inverse_inertia
            + r2tangent * r2tangent * body2.p.inverse_inertia;
        if denom_tangent > 0.0 {
            // Velocity after applying normal impulse
            let vel1_at_contact = body1.p.velocity + cross_scalar_vec(body1.p.angular_velocity, r1);
            let vel2_at_contact = body2.p.velocity + cross_scalar_vec(body2.p.angular_velocity, r2);
            let relative_velocity = vel2_at_contact - vel1_at_contact;

            let impulse_tangent_scalar = -relative_velocity.dot(&tangent) / denom_tangent;

            let friction_coefficient =
                0.5 * (body1.p.friction_coefficient + body2.p.friction_coefficient);
            let impulse_tangent_scalar = impulse_tangent_scalar.clamp(
                -friction_coefficient * impulse_scalar,
                friction_coefficient * impulse_scalar,
            );
            let impulse_tangent = tangent * impulse_tangent_scalar;

            // Apply linear impulses
            body1.p.velocity -= impulse_tangent * body1.p.inverse_mass;
            body2.p.velocity += impulse_tangent * body2.p.inverse_mass;
            // Apply angular impulses
            let tau1t = cross_vec_vec(r1, impulse_tangent);
            let tau2t = cross_vec_vec(r2, impulse_tangent);
            body1.p.angular_velocity -= body1.p.inverse_inertia * tau1t;
            body2.p.angular_velocity += body2.p.inverse_inertia * tau2t;
        }
    }
}

fn find_collisions(bodies: &[Body], collision_pair_indexes: &[(usize, usize)]) -> Vec<Collision> {
    let mut collisions = Vec::<Collision>::new();

    for &(body1_index, body2_index) in collision_pair_indexes {
        let (body1, body2) = (&bodies[body1_index], &bodies[body2_index]);

        // If both bodies have infinite mass, they should pass through each other
        if body1.p.inverse_mass == 0.0 && body2.p.inverse_mass == 0.0 {
            continue;
        };

        let collision_contact = match (&body1.shape, &body2.shape) {
            (&Shape::Circle { radius: radius1 }, &Shape::Circle { radius: radius2 }) => {
                collision_detection::circle_circle_collision(&body1.p, radius1, &body2.p, radius2)
            }
            (&Shape::Rectangle { size: size1 }, &Shape::Rectangle { size: size2 }) => {
                collision_detection::rectangle_rectangle_collision(&body1.p, size1, &body2.p, size2)
            }
            (
                &Shape::Rectangle {
                    size: rectangle_size,
                },
                &Shape::Circle {
                    radius: circle_radius,
                },
            ) => collision_detection::rectangle_circle_collision(
                &body1.p,
                rectangle_size,
                &body2.p,
                circle_radius,
            ),
            (
                &Shape::Circle {
                    radius: circle_radius,
                },
                &Shape::Rectangle {
                    size: rectangle_size,
                },
            ) => {
                let mut result = collision_detection::rectangle_circle_collision(
                    &body2.p,
                    rectangle_size,
                    &body1.p,
                    circle_radius,
                );
                // negate normal so it points from body 1 to body 2
                if let Some(ref mut contact) = result {
                    contact.normal *= -1.0;
                }
                result
            } // Not implemented
              // _ => None,
        };

        if let Some(contact) = collision_contact {
            collisions.push(Collision {
                body1_index,
                body2_index,
                contact,
            })
        }
    }
    collisions
}

fn body_pair_mut(
    bodies: &mut [Body],
    body1_index: usize,
    body2_index: usize,
) -> (&mut Body, &mut Body) {
    let (left, right) = bodies.split_at_mut(body2_index);
    (&mut left[body1_index], &mut right[0])
}
