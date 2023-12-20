pub mod vector;

pub use vector::{Float, Vec2};

#[derive(Clone)]
pub struct BodyProperties {
    pub position: Vec2,
    pub previous_position: Vec2,
    pub velocity: Vec2,
    pub previous_velocity: Vec2,
    pub reciprocal_mass: Float,
    pub restitution: Float,
    pub angle: Float,
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
    /// 1 divided by mass, can be 0
    pub reciprocal_mass: Float,
    pub restitution: Float,
    /// Anticlockwise angle in radians around centre
    pub angle: Float,

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
                reciprocal_mass: self.reciprocal_mass,
                restitution: self.restitution,
                angle: self.angle,
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

#[derive(Default, Clone)]
pub struct World {
    pub bodies: Vec<Body>,
    /// Acceleration downards
    pub gravity: Float,
    pub step_time: Float,
    pub substeps: u8,
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
}

impl World {
    pub fn new(delta_time: Float, substeps: u8, gravity: Float) -> World {
        World {
            bodies: vec![],
            gravity,
            step_time: delta_time,
            substeps,
        }
    }
    pub fn step(&mut self) {
        let collision_pair_indexes = {
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
                body.p.velocity.y += self.gravity * substep_time;
                body.p.position += body.p.velocity * substep_time;
            }

            let collisions = find_collisions(
                self.bodies.as_mut_slice(),
                collision_pair_indexes.as_slice(),
            );

            for body in self.bodies.iter_mut() {
                body.p.velocity = (body.p.position - body.p.previous_position) / substep_time;
            }

            resolve_collisions(self.bodies.as_mut_slice(), collisions.as_slice());
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
        let (body1, body2) = body_pair(bodies, body1_index, body2_index);

        // Move bodies by amounts proportional to their reciprocal masses so they no longer intersect
        let reciprocal_masses_sum = body1.p.reciprocal_mass + body2.p.reciprocal_mass;
        let body1_displacement_size =
            contact.intersection_depth * body1.p.reciprocal_mass / reciprocal_masses_sum;
        let body2_displacement_size =
            contact.intersection_depth * body2.p.reciprocal_mass / reciprocal_masses_sum;
        body1.p.position -= contact.normal * body1_displacement_size;
        body2.p.position += contact.normal * body2_displacement_size;

        // Update velocities
        let relative_normal_velocity = (body1.p.velocity - body2.p.velocity).dot(&contact.normal);
        let previous_relative_normal_velocity =
            (body1.p.previous_velocity - body2.p.previous_velocity).dot(&contact.normal);
        let restitution = (body1.p.restitution + body2.p.restitution) / 2.0;
        let relative_velocity_change = contact.normal
            * (relative_normal_velocity + restitution * previous_relative_normal_velocity);
        let reciprocal_masses_sum = body1.p.reciprocal_mass + body2.p.reciprocal_mass;
        body1.p.velocity -=
            relative_velocity_change / reciprocal_masses_sum * body1.p.reciprocal_mass;
        body2.p.velocity +=
            relative_velocity_change / reciprocal_masses_sum * body2.p.reciprocal_mass;
    }
}

fn find_collisions(
    bodies: &mut [Body],
    collision_pair_indexes: &[(usize, usize)],
) -> Vec<Collision> {
    let mut collisions = Vec::<Collision>::new();

    for &(body1_index, body2_index) in collision_pair_indexes {
        let (body1, body2) = body_pair(bodies, body1_index, body2_index);
        if body1.p.reciprocal_mass == 0.0 && body2.p.reciprocal_mass == 0.0 {
            continue;
        };
        let collision_contact = match (&body1.shape, &body2.shape) {
            (&Shape::Circle { radius: radius1 }, &Shape::Circle { radius: radius2 }) => {
                circle_circle_collision(&body1.p, radius1, &body2.p, radius2)
            }
            (&Shape::Rectangle { size: size1 }, &Shape::Rectangle { size: size2 }) => {
                rectangle_rectangle_collision(&body1.p, size1, &body2.p, size2)
            }
            _ => None,
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

fn body_pair(
    bodies: &mut [Body],
    body1_index: usize,
    body2_index: usize,
) -> (&mut Body, &mut Body) {
    let (left, right) = bodies.split_at_mut(body2_index);
    (&mut left[body1_index], &mut right[0])
}

// Assumes reciprocal masses are not 0
fn circle_circle_collision(
    p1: &BodyProperties,
    radius1: Float,
    p2: &BodyProperties,
    radius2: Float,
) -> Option<CollisionContact> {
    // Vector between centres of circles 1 -> 2
    let centre_1_to_2 = p2.position - p1.position;
    let radii_sum = radius1 + radius2;
    let radii_sum_squared = radii_sum.powi(2);

    // Check if intersecting using Pythagoras' theorem and return if they aren't
    if centre_1_to_2.length_squared() > radii_sum_squared {
        return None;
    }

    let distance_between_centres = centre_1_to_2.length();
    // Prevent division by 0
    let collision_normal = if distance_between_centres == 0.0 {
        Vec2::new(1.0, 0.0)
    } else {
        centre_1_to_2 / distance_between_centres
    };

    let intersection_depth = radii_sum - distance_between_centres;

    Some(CollisionContact {
        normal: collision_normal,
        intersection_depth,
    })
}

fn _rectangle_rectangle_collision_old(
    p1: &BodyProperties,
    size1: Vec2,
    p2: &BodyProperties,
    size2: Vec2,
) -> Option<CollisionContact> {
    let x_intersection = 0.5 * (size1.x + size2.x) - (p1.position.x - p2.position.x).abs();
    if x_intersection < 0.0 {
        return None;
    }

    let y_intersection = 0.5 * (size1.y + size2.y) - (p1.position.y - p2.position.y).abs();
    if y_intersection < 0.0 {
        return None;
    }

    let intersection_depth: Float;
    let collision_normal = if x_intersection < y_intersection {
        intersection_depth = x_intersection;
        if p1.position.x < p2.position.x {
            Vec2::new(1.0, 0.0)
        } else {
            Vec2::new(-1.0, 0.0)
        }
    } else {
        intersection_depth = y_intersection;
        if p1.position.y < p2.position.y {
            Vec2::new(0.0, 1.0)
        } else {
            Vec2::new(0.0, -1.0)
        }
    };

    Some(CollisionContact {
        normal: collision_normal,
        intersection_depth,
    })
}

fn rectangle_rectangle_collision(
    p1: &BodyProperties,
    size1: Vec2,
    p2: &BodyProperties,
    size2: Vec2,
) -> Option<CollisionContact> {
    // unit vectors rotated by angle of rectangle
    fn get_rotated_unit_vectors(angle: Float) -> (Vec2, Vec2) {
        let t = angle.sin_cos();
        (Vec2::new(t.1, t.0), Vec2::new(-t.0, t.1))
    }
    let (rotated_unit_x_1, rotated_unit_y_1) = get_rotated_unit_vectors(p1.angle);
    let (rotated_unit_x_2, rotated_unit_y_2) = get_rotated_unit_vectors(p2.angle);

    fn get_rectangle_vertices(position: Vec2, size: Vec2, rotated_unit_x: Vec2) -> [Vec2; 4] {
        [
            Vec2::new(1.0, 1.0),
            Vec2::new(-1.0, 1.0),
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
        ]
        .map(|corner| {
            let corner = Vec2::new(corner.x * 0.5 * size.x, corner.y * 0.5 * size.y);
            let corner = Vec2::new(
                rotated_unit_x.x * corner.x - rotated_unit_x.y * corner.y,
                rotated_unit_x.y * corner.x + rotated_unit_x.x * corner.y,
            );
            position + corner
        })
    }

    let vertices1 = get_rectangle_vertices(p1.position, size1, rotated_unit_x_1);
    let vertices2 = get_rectangle_vertices(p2.position, size2, rotated_unit_x_2);

    let mut collision_contact: Option<CollisionContact> = None;

    fn get_min_max_of_vertices_on_axis(points: &[Vec2], axis: Vec2) -> (Float, Float) {
        let mut min = points[0].dot(&axis);
        let mut max = min;
        for p in points.iter().skip(1) {
            let projection = axis.dot(p);
            if projection < min {
                min = projection;
            } else if projection > max {
                max = projection;
            }
        }
        (min, max)
    }

    fn check_intersection_on_axis(
        (min1, max1): (Float, Float),
        (min2, max2): (Float, Float),
        axis: Vec2,
        collision_contact: &mut Option<CollisionContact>,
    ) -> bool {
        if min1 > max2 || min2 > max1 {
            // Not intersecting
            *collision_contact = None;
            return false;
        }
        let intersection_depth;
        let collision_normal;
        if (max2 - min1) < (max1 - min2) {
            // Move body 1 in positive direction of axis
            collision_normal = Vec2::zero() - axis;
            intersection_depth = max2 - min1;
        } else {
            // Move body 2 in negative direction of axis
            collision_normal = axis;
            intersection_depth = max1 - min2;
        }
        let should_replace = collision_contact.is_none()
            || collision_contact
                .as_ref()
                .is_some_and(|c| c.intersection_depth > intersection_depth);
        if should_replace {
            *collision_contact = Some(CollisionContact {
                normal: collision_normal,
                intersection_depth,
            });
        };
        true
    }

    for axis in [
        rotated_unit_x_1,
        rotated_unit_y_1,
        rotated_unit_x_2,
        rotated_unit_y_2,
    ] {
        let m1 = get_min_max_of_vertices_on_axis(&vertices1, axis);
        let m2 = get_min_max_of_vertices_on_axis(&vertices2, axis);
        let should_continue = check_intersection_on_axis(m1, m2, axis, &mut collision_contact);
        if !should_continue {
            break;
        }
    }

    collision_contact
}
