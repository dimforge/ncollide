use nalgebra::na;
use nalgebra::na::{Cast, Indexable, FloatVecExt, Vec3, Vec2};
use procedural::{TriMesh, SplitIndexBuffer, Polyline};

/**
 * Generates a cuboid geometry with a split index buffer.
 *
 * # Arguments:
 * * `extents` - the extents of the cuboid.
 */
pub fn cuboid<N: Float + Clone + Cast<f64>>(extents: &Vec3<N>) -> TriMesh<N, Vec3<N>> {
    let mut cuboid = unit_cuboid();

    cuboid.scale_by(extents);

    cuboid
}

/**
 * Generates a cuboid geometry with a split index buffer.
 *
 * The cuboid is centered at the origin, and has its half extents set to 0.5.
 */
pub fn unit_cuboid<N: Float + Clone + Cast<f64>>() -> TriMesh<N, Vec3<N>> {
    let mut coords  = Vec::with_capacity(8);
    let mut uvs     = Vec::with_capacity(4);
    let mut normals = Vec::with_capacity(6);
    let mut faces   = Vec::with_capacity(12);

    let _0_5: N = na::cast(0.5);
    let m0_5: N = -_0_5;
    let _1:   N = na::one();
    let m1:   N = -_1;
    let _0:   N = na::zero();

    coords.push(Vec3::new(m0_5.clone(), m0_5.clone(), _0_5.clone()));
    coords.push(Vec3::new(m0_5.clone(), m0_5.clone(), m0_5.clone()));
    coords.push(Vec3::new(_0_5.clone(), m0_5.clone(), m0_5.clone()));
    coords.push(Vec3::new(_0_5.clone(), m0_5.clone(), _0_5.clone()));
    coords.push(Vec3::new(m0_5.clone(), _0_5.clone(), _0_5.clone()));
    coords.push(Vec3::new(m0_5.clone(), _0_5.clone(), m0_5.clone()));
    coords.push(Vec3::new(_0_5.clone(), _0_5.clone(), m0_5.clone()));
    coords.push(Vec3::new(_0_5.clone(), _0_5.clone(), _0_5.clone()));

    uvs.push(Vec2::new(_0.clone(), _1.clone()));
    uvs.push(Vec2::new(_1.clone(), _1.clone()));
    uvs.push(Vec2::new(_0.clone(), _0.clone()));
    uvs.push(Vec2::new(_1.clone(), _0.clone()));

    normals.push(Vec3::new(m1.clone(), _0.clone(), _0.clone()));
    normals.push(Vec3::new(_0.clone(), _0.clone(), m1.clone()));
    normals.push(Vec3::new(_1.clone(), _0.clone(), _0.clone()));
    normals.push(Vec3::new(_0.clone(), _0.clone(), _1.clone()));
    normals.push(Vec3::new(_0.clone(), m1.clone(), _0.clone()));
    normals.push(Vec3::new(_0.clone(), _1.clone(), _0.clone()));

    faces.push(Vec3::new(Vec3::new(4, 0, 0), Vec3::new(5, 1, 0), Vec3::new(0, 2, 0)));
    faces.push(Vec3::new(Vec3::new(5, 0, 1), Vec3::new(6, 1, 1), Vec3::new(1, 2, 1)));
    faces.push(Vec3::new(Vec3::new(6, 1, 2), Vec3::new(7, 0, 2), Vec3::new(3, 2, 2)));
    faces.push(Vec3::new(Vec3::new(7, 1, 3), Vec3::new(4, 0, 3), Vec3::new(0, 2, 3)));
    faces.push(Vec3::new(Vec3::new(0, 2, 4), Vec3::new(1, 0, 4), Vec3::new(2, 1, 4)));
    faces.push(Vec3::new(Vec3::new(7, 3, 5), Vec3::new(6, 1, 5), Vec3::new(5, 0, 5)));
    faces.push(Vec3::new(Vec3::new(5, 1, 0), Vec3::new(1, 3, 0), Vec3::new(0, 2, 0)));
    faces.push(Vec3::new(Vec3::new(6, 1, 1), Vec3::new(2, 3, 1), Vec3::new(1, 2, 1)));
    faces.push(Vec3::new(Vec3::new(2, 3, 2), Vec3::new(6, 1, 2), Vec3::new(3, 2, 2)));
    faces.push(Vec3::new(Vec3::new(3, 3, 3), Vec3::new(7, 1, 3), Vec3::new(0, 2, 3)));
    faces.push(Vec3::new(Vec3::new(3, 3, 4), Vec3::new(0, 2, 4), Vec3::new(2, 1, 4)));
    faces.push(Vec3::new(Vec3::new(4, 2, 5), Vec3::new(7, 3, 5), Vec3::new(5, 0, 5)));

    TriMesh::new(coords, Some(normals), Some(uvs), Some(SplitIndexBuffer(faces)))
}

/// The contour of a cuboid lying on the x-y plane.
pub fn rectangle<N: Float + Clone + Cast<f64>, V: FloatVecExt<N>>(extents: &V) -> Polyline<N, V> {
    let mut rectangle = unit_rectangle();

    rectangle.scale_by(extents);

    rectangle
}

/// The contour of a unit cuboid lying on the x-y plane.
pub fn unit_rectangle<N: Float + Clone + Cast<f64>, V: FloatVecExt<N>>() -> Polyline<N, V> {
    let _0_5: N = na::cast(0.5);
    let m0_5: N = -_0_5;

    let mut p_ul = na::zero::<V>();
    let mut p_ur = na::zero::<V>();
    let mut p_dl = na::zero::<V>();
    let mut p_dr = na::zero::<V>();

    p_ul.set(0, m0_5.clone()); p_ul.set(1, _0_5.clone());
    p_ur.set(0, _0_5.clone()); p_ur.set(1, _0_5.clone());
    p_dl.set(0, _0_5.clone()); p_dl.set(1, m0_5.clone());
    p_dr.set(0, m0_5.clone()); p_dr.set(1, m0_5.clone());

    Polyline::new(vec!(p_ur, p_ul, p_dl, p_dr), None)
}
