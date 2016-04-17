use na;
use na::{Vector3, Point2, Point3};
use super::{TriMesh, IndexBuffer, Polyline};
use math::{Scalar, Point, Vector};

/**
 * Generates a cuboid shape with a split index buffer.
 *
 * # Arguments:
 * * `extents` - the extents of the cuboid.
 */
pub fn cuboid<N>(extents: &Vector3<N>) -> TriMesh<Point3<N>>
    where N: Scalar {
    let mut cuboid = unit_cuboid();

    cuboid.scale_by(extents);

    cuboid
}

/**
 * Generates a cuboid shape with a split index buffer.
 *
 * The cuboid is centered at the origin, and has its half extents set to 0.5.
 */
pub fn unit_cuboid<N>() -> TriMesh<Point3<N>>
    where N: Scalar {
    let mut coords  = Vec::with_capacity(8);
    let mut uvs     = Vec::with_capacity(4);
    let mut normals = Vec::with_capacity(6);
    let mut faces   = Vec::with_capacity(12);

    let _0_5: N = na::cast(0.5);
    let m0_5: N = -_0_5;
    let _1:   N = na::one();
    let m1:   N = -_1;
    let _0:   N = na::zero();

    coords.push(Point3::new(m0_5.clone(), m0_5.clone(), _0_5.clone()));
    coords.push(Point3::new(m0_5.clone(), m0_5.clone(), m0_5.clone()));
    coords.push(Point3::new(_0_5.clone(), m0_5.clone(), m0_5.clone()));
    coords.push(Point3::new(_0_5.clone(), m0_5.clone(), _0_5.clone()));
    coords.push(Point3::new(m0_5.clone(), _0_5.clone(), _0_5.clone()));
    coords.push(Point3::new(m0_5.clone(), _0_5.clone(), m0_5.clone()));
    coords.push(Point3::new(_0_5.clone(), _0_5.clone(), m0_5.clone()));
    coords.push(Point3::new(_0_5.clone(), _0_5.clone(), _0_5.clone()));

    uvs.push(Point2::new(_0.clone(), _1.clone()));
    uvs.push(Point2::new(_1.clone(), _1.clone()));
    uvs.push(Point2::new(_0.clone(), _0.clone()));
    uvs.push(Point2::new(_1.clone(), _0.clone()));

    normals.push(Vector3::new(m1.clone(), _0.clone(), _0.clone()));
    normals.push(Vector3::new(_0.clone(), _0.clone(), m1.clone()));
    normals.push(Vector3::new(_1.clone(), _0.clone(), _0.clone()));
    normals.push(Vector3::new(_0.clone(), _0.clone(), _1.clone()));
    normals.push(Vector3::new(_0.clone(), m1.clone(), _0.clone()));
    normals.push(Vector3::new(_0.clone(), _1.clone(), _0.clone()));

    faces.push(Point3::new(Point3::new(4, 0, 0), Point3::new(5, 0, 1), Point3::new(0, 0, 2)));
    faces.push(Point3::new(Point3::new(5, 0, 1), Point3::new(1, 0, 3), Point3::new(0, 0, 2)));

    faces.push(Point3::new(Point3::new(5, 1, 0), Point3::new(6, 1, 1), Point3::new(1, 1, 2)));
    faces.push(Point3::new(Point3::new(6, 1, 1), Point3::new(2, 1, 3), Point3::new(1, 1, 2)));

    faces.push(Point3::new(Point3::new(6, 2, 1), Point3::new(7, 2, 0), Point3::new(3, 2, 2)));
    faces.push(Point3::new(Point3::new(2, 2, 3), Point3::new(6, 2, 1), Point3::new(3, 2, 2)));

    faces.push(Point3::new(Point3::new(7, 3, 1), Point3::new(4, 3, 0), Point3::new(0, 3, 2)));
    faces.push(Point3::new(Point3::new(3, 3, 3), Point3::new(7, 3, 1), Point3::new(0, 3, 2)));

    faces.push(Point3::new(Point3::new(0, 4, 2), Point3::new(1, 4, 0), Point3::new(2, 4, 1)));
    faces.push(Point3::new(Point3::new(3, 4, 3), Point3::new(0, 4, 2), Point3::new(2, 4, 1)));

    faces.push(Point3::new(Point3::new(7, 5, 3), Point3::new(6, 5, 1), Point3::new(5, 5, 0)));
    faces.push(Point3::new(Point3::new(4, 5, 2), Point3::new(7, 5, 3), Point3::new(5, 5, 0)));

    TriMesh::new(coords, Some(normals), Some(uvs), Some(IndexBuffer::Split(faces)))
}

/// The contour of a cuboid lying on the x-y plane.
pub fn rectangle<P>(extents: &P::Vect) -> Polyline<P>
    where P: Point {
    let mut rectangle = unit_rectangle::<P>();

    rectangle.scale_by(extents);

    rectangle
}

/// The contour of a unit cuboid lying on the x-y plane.
pub fn unit_rectangle<P>() -> Polyline<P>
    where P: Point {
    let _0_5: <P::Vect as Vector>::Scalar = na::cast(0.5);
    let m0_5: <P::Vect as Vector>::Scalar = -_0_5;

    let mut p_ul = na::origin::<P>();
    let mut p_ur = na::origin::<P>();
    let mut p_dl = na::origin::<P>();
    let mut p_dr = na::origin::<P>();

    p_dl[0] = m0_5.clone(); p_dl[1] = m0_5.clone();
    p_dr[0] = _0_5.clone(); p_dr[1] = m0_5.clone();
    p_ur[0] = _0_5.clone(); p_ur[1] = _0_5.clone();
    p_ul[0] = m0_5.clone(); p_ul[1] = _0_5.clone();

    Polyline::new(vec!(p_ur, p_ul, p_dl, p_dr), None)
}
