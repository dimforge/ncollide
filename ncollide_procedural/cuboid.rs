use na;
use na::{Vec3, Pnt2, Pnt3};
use super::{TriMesh, IndexBuffer, Polyline};
use math::{Scalar, Point, Vect};

/**
 * Generates a cuboid shape with a split index buffer.
 *
 * # Arguments:
 * * `extents` - the extents of the cuboid.
 */
pub fn cuboid<N>(extents: &Vec3<N>) -> TriMesh<Pnt3<N>>
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
pub fn unit_cuboid<N>() -> TriMesh<Pnt3<N>>
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

    coords.push(Pnt3::new(m0_5.clone(), m0_5.clone(), _0_5.clone()));
    coords.push(Pnt3::new(m0_5.clone(), m0_5.clone(), m0_5.clone()));
    coords.push(Pnt3::new(_0_5.clone(), m0_5.clone(), m0_5.clone()));
    coords.push(Pnt3::new(_0_5.clone(), m0_5.clone(), _0_5.clone()));
    coords.push(Pnt3::new(m0_5.clone(), _0_5.clone(), _0_5.clone()));
    coords.push(Pnt3::new(m0_5.clone(), _0_5.clone(), m0_5.clone()));
    coords.push(Pnt3::new(_0_5.clone(), _0_5.clone(), m0_5.clone()));
    coords.push(Pnt3::new(_0_5.clone(), _0_5.clone(), _0_5.clone()));

    uvs.push(Pnt2::new(_0.clone(), _1.clone()));
    uvs.push(Pnt2::new(_1.clone(), _1.clone()));
    uvs.push(Pnt2::new(_0.clone(), _0.clone()));
    uvs.push(Pnt2::new(_1.clone(), _0.clone()));

    normals.push(Vec3::new(m1.clone(), _0.clone(), _0.clone()));
    normals.push(Vec3::new(_0.clone(), _0.clone(), m1.clone()));
    normals.push(Vec3::new(_1.clone(), _0.clone(), _0.clone()));
    normals.push(Vec3::new(_0.clone(), _0.clone(), _1.clone()));
    normals.push(Vec3::new(_0.clone(), m1.clone(), _0.clone()));
    normals.push(Vec3::new(_0.clone(), _1.clone(), _0.clone()));

    faces.push(Pnt3::new(Pnt3::new(4, 0, 0), Pnt3::new(5, 0, 1), Pnt3::new(0, 0, 2)));
    faces.push(Pnt3::new(Pnt3::new(5, 0, 1), Pnt3::new(1, 0, 3), Pnt3::new(0, 0, 2)));

    faces.push(Pnt3::new(Pnt3::new(5, 1, 0), Pnt3::new(6, 1, 1), Pnt3::new(1, 1, 2)));
    faces.push(Pnt3::new(Pnt3::new(6, 1, 1), Pnt3::new(2, 1, 3), Pnt3::new(1, 1, 2)));

    faces.push(Pnt3::new(Pnt3::new(6, 2, 1), Pnt3::new(7, 2, 0), Pnt3::new(3, 2, 2)));
    faces.push(Pnt3::new(Pnt3::new(2, 2, 3), Pnt3::new(6, 2, 1), Pnt3::new(3, 2, 2)));

    faces.push(Pnt3::new(Pnt3::new(7, 3, 1), Pnt3::new(4, 3, 0), Pnt3::new(0, 3, 2)));
    faces.push(Pnt3::new(Pnt3::new(3, 3, 3), Pnt3::new(7, 3, 1), Pnt3::new(0, 3, 2)));

    faces.push(Pnt3::new(Pnt3::new(0, 4, 2), Pnt3::new(1, 4, 0), Pnt3::new(2, 4, 1)));
    faces.push(Pnt3::new(Pnt3::new(3, 4, 3), Pnt3::new(0, 4, 2), Pnt3::new(2, 4, 1)));

    faces.push(Pnt3::new(Pnt3::new(7, 5, 3), Pnt3::new(6, 5, 1), Pnt3::new(5, 5, 0)));
    faces.push(Pnt3::new(Pnt3::new(4, 5, 2), Pnt3::new(7, 5, 3), Pnt3::new(5, 5, 0)));

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
    let _0_5: <P::Vect as Vect>::Scalar = na::cast(0.5);
    let m0_5: <P::Vect as Vect>::Scalar = -_0_5;

    let mut p_ul = na::orig::<P>();
    let mut p_ur = na::orig::<P>();
    let mut p_dl = na::orig::<P>();
    let mut p_dr = na::orig::<P>();

    p_dl[0] = m0_5.clone(); p_dl[1] = m0_5.clone();
    p_dr[0] = _0_5.clone(); p_dr[1] = m0_5.clone();
    p_ur[0] = _0_5.clone(); p_ur[1] = _0_5.clone();
    p_ul[0] = m0_5.clone(); p_ul[1] = _0_5.clone();

    Polyline::new(vec!(p_ur, p_ul, p_dl, p_dr), None)
}
