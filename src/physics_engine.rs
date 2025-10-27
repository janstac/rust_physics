pub mod vector;

pub use vector::{Float, Vec2};

#[derive(Clone)]
pub struct BodyProperties {
    pub position: Vec2,
    pub previous_position: Vec2,
    pub velocity: Vec2,
    pub previous_velocity: Vec2,
    pub acceleration: Vec2,
    pub reciprocal_mass: Float,
    pub restitution: Float,
    pub angle: Float,
    pub angular_velocity: Float,
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
    pub reciprocal_mass: Float,
    pub restitution: Float,
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

                reciprocal_mass: self.reciprocal_mass,
                restitution: self.restitution,
                angle: self.angle,
                angular_velocity: self.angular_velocity,
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

            for collision in collisions.iter() {
                self.contacts.push(collision.contact.contact_point); // TODO temp
            }

            resolve_collisions(self.bodies.as_mut_slice(), collisions.as_slice());
        }
    }
}

fn cross_vec_vec(a: Vec2, b: Vec2) -> Float {
    // (ax, ay, 0) x (bx, by, 0) = (0,0, z)
    a.x * b.y - a.y * b.x
}
fn cross_scalar_vec(v: Vec2, s: Float) -> Vec2 {
    // (vx, vy, 0) x (0,0,s)
    Vec2::new(-s * v.y, s * v.x)
}

// Compute inverse moment of inertia from shape and inverse mass
fn inv_inertia(body: &Body) -> Float {
    let inv_m = body.p.reciprocal_mass;
    if inv_m == 0.0 {
        return 0.0;
    }
    match body.shape {
        Shape::Circle { radius } => {
            // I = 0.5 * m * r^2  => invI = 2 * inv_m / r^2
            if radius <= 0.0 {
                0.0
            } else {
                2.0 * inv_m / (radius * radius)
            }
        }
        Shape::Rectangle { size } => {
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

        // Move bodies by amounts proportional to their reciprocal masses so they no longer intersect
        let reciprocal_masses_sum = body1.p.reciprocal_mass + body2.p.reciprocal_mass;

        let body1_displacement_size =
            contact.intersection_depth * body1.p.reciprocal_mass / reciprocal_masses_sum;
        let body2_displacement_size =
            contact.intersection_depth * body2.p.reciprocal_mass / reciprocal_masses_sum;

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
        let vel1_at_contact = body1.p.velocity + cross_scalar_vec(r1, body1.p.angular_velocity);
        let vel2_at_contact = body2.p.velocity + cross_scalar_vec(r2, body2.p.angular_velocity);

        // >0 means separating
        let relative_velocity_along_normal =
            (vel2_at_contact - vel1_at_contact).dot(&contact.normal);

        // Only resolve if bodies are approaching along normal
        if relative_velocity_along_normal > 0.0 {
            continue;
        }

        // Average restitution
        let restitution = (body1.p.restitution + body2.p.restitution) / 2.0;

        // TODO: understand this

        // Rotational contribution: (r × n)^2 * invI
        let r1n = cross_vec_vec(r1, contact.normal);
        let r2n = cross_vec_vec(r2, contact.normal);
        let inv_inertia1 = inv_inertia(body1);
        let inv_inertia2 = inv_inertia(body2);
        let inv_inertia_sum = r1n * r1n * inv_inertia1 + r2n * r2n * inv_inertia2;

        let denom = reciprocal_masses_sum + inv_inertia_sum;

        let impulse_scalar = -(1.0 + restitution) * relative_velocity_along_normal / denom;
        let impulse = contact.normal * impulse_scalar;

        // Apply linear impulses
        body1.p.velocity -= impulse * body1.p.reciprocal_mass;
        body2.p.velocity += impulse * body2.p.reciprocal_mass;

        // Apply angular impulses (Δω = invI * τ, τ = r × F; body1 gets -impulse, body2 gets +impulse)
        let tau1 = cross_vec_vec(r1, impulse);
        let tau2 = cross_vec_vec(r2, impulse);
        body1.p.angular_velocity -= inv_inertia1 * tau1;
        body2.p.angular_velocity += inv_inertia2 * tau2;
    }
}

fn find_collisions(
    // TODO: why did I make this mut?
    bodies: &mut [Body],
    collision_pair_indexes: &[(usize, usize)],
) -> Vec<Collision> {
    let mut collisions = Vec::<Collision>::new();

    for &(body1_index, body2_index) in collision_pair_indexes {
        let (body1, body2) = body_pair_mut(bodies, body1_index, body2_index);

        // If both bodies have infinite mass, they should pass through each other
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
            (
                &Shape::Rectangle {
                    size: rectangle_size,
                },
                &Shape::Circle {
                    radius: circle_radius,
                },
            ) => rectangle_circle_collision(&body1.p, rectangle_size, &body2.p, circle_radius),
            (
                &Shape::Circle {
                    radius: circle_radius,
                },
                &Shape::Rectangle {
                    size: rectangle_size,
                },
            ) => {
                let mut result =
                    rectangle_circle_collision(&body2.p, rectangle_size, &body1.p, circle_radius);
                // negate normal
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
        // Circles have the exact same position, so any unit vector could be used as the normal
        Vec2::new(1.0, 0.0)
    } else {
        centre_1_to_2 / distance_between_centres
    };

    let intersection_depth = radii_sum - distance_between_centres;

    // Approximate as midpoint of overlap
    let contact_point = p1.position + collision_normal * (radius1 - intersection_depth / 2.0);

    Some(CollisionContact {
        normal: collision_normal,
        intersection_depth,
        contact_point,
    })
}

fn rectangle_circle_collision(
    p_rectangle: &BodyProperties,
    rectangle_size: Vec2,
    p_circle: &BodyProperties,
    circle_radius: Float,
) -> Option<CollisionContact> {
    // Checks if rectangles are colliding by using the separating axis theorem

    // x & y unit vectors rotated by angle of rectangle
    let (rotated_unit_x_1, rotated_unit_y_1) = get_rotated_unit_vectors(p_rectangle.angle);

    let vertices1 = get_rectangle_vertices(p_rectangle.position, rectangle_size, rotated_unit_x_1);

    let mut collision_contact: Option<CollisionContact> = None;

    let circle_centre_to_closest_vertex = {
        let centre = p_circle.position;
        let mut shortest_vector = vertices1[0] - centre;
        let mut closest_distance_squared = shortest_vector.length_squared();
        for vertex in vertices1.iter().skip(1) {
            let vector = *vertex - centre;
            let distance_squared = vector.length_squared();
            if distance_squared < closest_distance_squared {
                shortest_vector = vector;
                closest_distance_squared = distance_squared;
            }
        }
        shortest_vector
    };

    let axis3 = circle_centre_to_closest_vertex.normalised();

    for (axis, axis_is_from_body1) in [
        (rotated_unit_x_1, true),
        (rotated_unit_y_1, true),
        (axis3, false),
    ] {
        let minmax1 = get_min_max_projections_of_vertices_on_axis(&vertices1, axis);
        let minmax2 = {
            let min_point = p_circle.position - axis * circle_radius;
            let max_point = p_circle.position + axis * circle_radius;
            MinMaxProjections {
                min: min_point.dot(&axis),
                max: max_point.dot(&axis),
                min_point,
                max_point,
            }
        };
        let should_continue = check_intersection_on_axis(
            minmax1,
            minmax2,
            axis,
            &mut collision_contact,
            axis_is_from_body1,
        );

        // If there is an axis on which there is no overlap, rectangles are not intersecting
        if !should_continue {
            break;
        }
    }

    collision_contact
}

// unit vectors rotated by angle of rectangle
fn get_rotated_unit_vectors(angle: Float) -> (Vec2, Vec2) {
    let (s, c) = angle.sin_cos();
    (Vec2::new(c, s), Vec2::new(-s, c))
}

fn get_rectangle_vertices(position: Vec2, size: Vec2, rotated_unit_x: Vec2) -> [Vec2; 4] {
    [
        // Vertices of a unit square centered at origin
        Vec2::new(0.5, 0.5),
        Vec2::new(-0.5, 0.5),
        Vec2::new(-0.5, -0.5),
        Vec2::new(0.5, -0.5),
    ]
    .map(|corner| {
        // Scale position by size of rectangle
        let corner = Vec2::new(corner.x * size.x, corner.y * size.y);
        // Rotate it by angle
        let corner = Vec2::new(
            rotated_unit_x.x * corner.x - rotated_unit_x.y * corner.y,
            rotated_unit_x.y * corner.x + rotated_unit_x.x * corner.y,
        );
        position + corner
    })
}

struct MinMaxProjections {
    min: Float,
    max: Float,
    min_point: Vec2,
    max_point: Vec2,
}
// Project each point onto axis vector, and find the minimum and maximum projections
fn get_min_max_projections_of_vertices_on_axis(points: &[Vec2], axis: Vec2) -> MinMaxProjections {
    // Initialise with projections of first point
    let mut min_point = points[0];
    let mut min = min_point.dot(&axis);
    let mut max_point = min_point;
    let mut max = min;

    for p in points.iter().skip(1) {
        let projection = axis.dot(p);
        if projection < min {
            min = projection;
            min_point = *p;
        } else if projection > max {
            max = projection;
            max_point = *p;
        }
    }
    MinMaxProjections {
        min,
        max,
        min_point,
        max_point,
    }
}

// Check if both (min,max) ranges overlap, and if so, return true, and replace the collision contact if
// this new overlap is smaller
fn check_intersection_on_axis(
    minmax1: MinMaxProjections,
    minmax2: MinMaxProjections,
    axis: Vec2,
    collision_contact: &mut Option<CollisionContact>,
    axis_is_from_body1: bool,
) -> bool {
    let min1 = minmax1.min;
    let max1 = minmax1.max;
    let min2 = minmax2.min;
    let max2 = minmax2.max;

    if min1 > max2 || min2 > max1 {
        // Not intersecting
        *collision_contact = None;
        return false;
    }
    // There is overlap on the axis

    let intersection_depth;
    let collision_normal;
    let contact_point;

    // Check which way the overlap is smaller
    if (max2 - min1) < (max1 - min2) {
        // 1:           |--------|  ->
        // 2: <-  |-------|
        collision_normal = Vec2::zero() - axis;
        intersection_depth = max2 - min1;

        let half_overlap_vector = collision_normal * intersection_depth / 2.0;
        contact_point = if axis_is_from_body1 {
            minmax2.max_point + half_overlap_vector
        } else {
            minmax1.min_point - half_overlap_vector
        };
    } else {
        // 1: <-  |--------|
        // 2:            |-------|  ->
        collision_normal = axis;
        intersection_depth = max1 - min2;

        let half_overlap_vector = collision_normal * intersection_depth / 2.0;
        contact_point = if axis_is_from_body1 {
            minmax2.min_point + half_overlap_vector
        } else {
            minmax1.max_point - half_overlap_vector
        };
    }

    // Should replace existing collision contact if there isn't one or if a smaller intersection depth was found
    let should_replace = collision_contact.is_none()
        || collision_contact
            .as_ref()
            .is_some_and(|c| c.intersection_depth > intersection_depth);
    if should_replace {
        *collision_contact = Some(CollisionContact {
            normal: collision_normal,
            intersection_depth,
            contact_point,
        });
    };
    true
}

fn rectangle_rectangle_collision(
    p1: &BodyProperties,
    size1: Vec2,
    p2: &BodyProperties,
    size2: Vec2,
) -> Option<CollisionContact> {
    // Checks if rectangles are colliding by using the separating axis theorem

    // Pairs of x & y unit vectors rotated by angles of rectangles 1 and 2
    let (rotated_unit_x_1, rotated_unit_y_1) = get_rotated_unit_vectors(p1.angle);
    let (rotated_unit_x_2, rotated_unit_y_2) = get_rotated_unit_vectors(p2.angle);

    let vertices1 = get_rectangle_vertices(p1.position, size1, rotated_unit_x_1);
    let vertices2 = get_rectangle_vertices(p2.position, size2, rotated_unit_x_2);

    let mut collision_contact: Option<CollisionContact> = None;

    for (axis, axis_is_from_body1) in [
        (rotated_unit_x_1, true),
        (rotated_unit_y_1, true),
        (rotated_unit_x_2, false),
        (rotated_unit_y_2, false),
    ] {
        let minmax1 = get_min_max_projections_of_vertices_on_axis(&vertices1, axis);
        let minmax2 = get_min_max_projections_of_vertices_on_axis(&vertices2, axis);
        let should_continue = check_intersection_on_axis(
            minmax1,
            minmax2,
            axis,
            &mut collision_contact,
            axis_is_from_body1,
        );

        // If there is an axis on which there is no overlap, rectangles are not intersecting
        if !should_continue {
            break;
        }
    }

    collision_contact
}
