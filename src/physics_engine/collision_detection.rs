use super::{BodyProperties, CollisionContact, Float, Vec2};

pub fn circle_circle_collision(
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

pub fn rectangle_circle_collision(
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

pub fn rectangle_rectangle_collision(
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
