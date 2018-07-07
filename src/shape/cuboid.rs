//! Support mapping based Cuboid shape.

use std::f64;
#[cfg(feature = "dim3")]
use smallvec::SmallVec;

use na::{self, Real, Unit};
use utils::IsometryOps;
use bounding_volume::PolyhedralCone;
use shape::{ConvexPolygonalFeature, ConvexPolyhedron, FeatureId, SupportMap};
use math::{Isometry, Point, Vector, DIM};

/// Shape of a box.
#[derive(PartialEq, Debug, Clone)]
pub struct Cuboid<N: Real> {
    half_extents: Vector<N>,
}

// NOTE: format of the cuboid feature id:
//
// FeatureId::Vertex(id): the i-th bit of `id` is set to 1 iff. the i-th component of the vertex is negative.
// FeatureId::Edge(id): the part `id & 0b11` contains a number in [0,1] (or [0,2] in 3D) to indicate the axis (x, y, z).
//                      the part `id >> 2` follow the same rule as the vertex id.
// FeatureId::Face(id): if `id` lies in [0,1] (or [0 2] in 3D) indicates the axis (x, y, z) corresponding to the face normal.
//                      If `id` is greater than 1 (or 2 in 3D), then the negative axis (-x, -y, -z) is given by `id - 2` (or `id - 3` in 3D).
impl<N: Real> Cuboid<N> {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be greater than 0.04.
    #[inline]
    pub fn new(half_extents: Vector<N>) -> Cuboid<N> {
        for i in 0..DIM {
            assert!(half_extents[i] >= N::zero());
        }

        Cuboid {
            half_extents: half_extents,
        }
    }
}

impl<N: Real> Cuboid<N> {
    /// The half-extents of this box. Half-extents are the box half-width along each axis.
    #[inline]
    pub fn half_extents(&self) -> &Vector<N> {
        &self.half_extents
    }
}

impl<N: Real> SupportMap<N> for Cuboid<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        let mut res = *self.half_extents();

        for i in 0usize..DIM {
            if dir[i] < N::zero() {
                res[i] = -res[i];
            }
        }

        Point::from_coordinates(res)
    }
}

impl<N: Real> ConvexPolyhedron<N> for Cuboid<N> {
    fn vertex(&self, id: FeatureId) -> Point<N> {
        let vid = id.unwrap_vertex();
        let mut res = self.half_extents;

        for i in 0..DIM {
            if vid & (1 << i) != 0 {
                res[i] = -res[i]
            }
        }

        Point::from_coordinates(res)
    }

    #[cfg(feature = "dim3")]
    fn edge(&self, id: FeatureId) -> (Point<N>, Point<N>, FeatureId, FeatureId) {
        let eid = id.unwrap_edge();
        let mut res = self.half_extents;

        let edge_i = eid & 0b11;
        let vertex_i = eid >> 2;

        for i in 0..DIM {
            if i != edge_i && (vertex_i & (1 << i) != 0) {
                res[i] = -res[i]
            }
        }

        let p1 = Point::from_coordinates(res);
        res[edge_i] = -res[edge_i];
        let p2 = Point::from_coordinates(res);
        let vid1 = FeatureId::Vertex(vertex_i & !(1 << edge_i));
        let vid2 = FeatureId::Vertex(vertex_i | (1 << edge_i));

        (p1, p2, vid1, vid2)
    }

    fn face(&self, id: FeatureId, out: &mut ConvexPolygonalFeature<N>) {
        out.clear();

        let i = id.unwrap_face();
        let i1;
        let sign;

        if i < DIM {
            i1 = i;
            sign = N::one();
        } else {
            i1 = i - DIM;
            sign = -N::one();
        }

        #[cfg(feature = "dim2")]
        {
            let i2 = (i1 + 1) % 2;

            let mut vertex = self.half_extents;
            vertex[i1] *= sign;
            vertex[i2] *= if i1 == 0 { -sign } else { sign };

            let p1 = Point::from_coordinates(vertex);
            vertex[i2] = -vertex[i2];
            let p2 = Point::from_coordinates(vertex);

            let mut vertex_id1 = if sign < na::zero() { 1 << i1 } else { 0 };
            let mut vertex_id2 = vertex_id1;
            if p1[i2] < na::zero() {
                vertex_id1 |= 1 << i2;
            } else {
                vertex_id2 |= 1 << i2;
            }

            out.push(p1, FeatureId::Vertex(vertex_id1));
            out.push(p2, FeatureId::Vertex(vertex_id2));

            let mut normal: Vector<N> = na::zero();
            normal[i1] = sign;
            out.set_normal(Unit::new_unchecked(normal));
            out.set_feature_id(FeatureId::Face(i));
        }
        #[cfg(feature = "dim3")]
        {
            let i2 = (i1 + 1) % 3;
            let i3 = (i1 + 2) % 3;
            let (edge_i2, edge_i3) = if sign > na::zero() {
                (i2, i3)
            } else {
                (i3, i2)
            };
            let mut vertex = self.half_extents;
            vertex[i1] *= sign;

            let (sbit, msbit) = if sign < na::zero() { (1, 0) } else { (0, 1) };
            let mut vertex_id = sbit << i1;
            out.push(Point::from_coordinates(vertex), FeatureId::Vertex(vertex_id));
            out.push_edge_feature_id(FeatureId::Edge(edge_i2 | (vertex_id << 2)));

            vertex[i2] = -sign * self.half_extents[i2];
            vertex[i3] = sign * self.half_extents[i3];
            vertex_id |= msbit << i2 | sbit << i3;
            out.push(Point::from_coordinates(vertex), FeatureId::Vertex(vertex_id));
            out.push_edge_feature_id(FeatureId::Edge(edge_i3 | (vertex_id << 2)));

            vertex[i2] = -self.half_extents[i2];
            vertex[i3] = -self.half_extents[i3];
            vertex_id |= 1 << i2 | 1 << i3;
            out.push(Point::from_coordinates(vertex), FeatureId::Vertex(vertex_id));
            out.push_edge_feature_id(FeatureId::Edge(edge_i2 | (vertex_id << 2)));

            vertex[i2] = sign * self.half_extents[i2];
            vertex[i3] = -sign * self.half_extents[i3];
            vertex_id = sbit << i1 | sbit << i2 | msbit << i3;
            out.push(Point::from_coordinates(vertex), FeatureId::Vertex(vertex_id));
            out.push_edge_feature_id(FeatureId::Edge(edge_i3 | (vertex_id << 2)));

            let mut normal: Vector<N> = na::zero();
            normal[i1] = sign;
            out.set_normal(Unit::new_unchecked(normal));

            if sign > na::zero() {
                out.set_feature_id(FeatureId::Face(i1));
            } else {
                out.set_feature_id(FeatureId::Face(i1 + 3));
            }

            out.recompute_edge_normals();
        }
    }

    fn support_face_toward(&self, m: &Isometry<N>, dir: &Unit<Vector<N>>, out: &mut ConvexPolygonalFeature<N>) {
        out.clear();
        let local_dir = m.inverse_transform_vector(dir);

        let mut iamax = 0;
        let mut amax = local_dir[0].abs();

        // FIXME: we should use nalgebra's iamax method.
        for i in 1..DIM {
            let candidate = local_dir[i].abs();
            if candidate > amax {
                amax = candidate;
                iamax = i;
            }
        }

        if local_dir[iamax] > na::zero() {
            self.face(FeatureId::Face(iamax), out);
            out.transform_by(m);
        } else {
            self.face(FeatureId::Face(iamax + DIM), out);
            out.transform_by(m);
        }
    }

    fn support_feature_toward(
        &self,
        m: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        angle: N,
        out: &mut ConvexPolygonalFeature<N>,
    ) {
        let local_dir = m.inverse_transform_vector(dir);
        let cang = angle.cos();
        let mut support_point = self.half_extents;

        out.clear();

        #[cfg(feature = "dim2")]
        {
            let mut support_point_id = 0;
            for i1 in 0..2 {
                let sign = local_dir[i1].signum();
                if sign * local_dir[i1] >= cang {
                    if sign > na::zero() {
                        self.face(FeatureId::Face(i1), out);
                        out.transform_by(m);
                    } else {
                        self.face(FeatureId::Face(i1 + 2), out);
                        out.transform_by(m);
                    }
                    return;
                } else {
                    if sign < na::zero() {
                        support_point_id |= 1 << i1;
                    }
                    support_point[i1] *= sign;
                }
            }

            // We are not on a face, return the support vertex.
            out.push(
                m * Point::from_coordinates(support_point),
                FeatureId::Vertex(support_point_id),
            );
            out.set_feature_id(FeatureId::Vertex(support_point_id));
        }

        #[cfg(feature = "dim3")]
        {
            let sang = angle.sin();
            let mut support_point_id = 0;

            // Check faces.
            for i1 in 0..3 {
                let sign = local_dir[i1].signum();
                if sign * local_dir[i1] >= cang {
                    if sign > na::zero() {
                        self.face(FeatureId::Face(i1), out);
                        out.transform_by(m);
                    } else {
                        self.face(FeatureId::Face(i1 + 3), out);
                        out.transform_by(m);
                    }
                    return;
                } else {
                    if sign < na::zero() {
                        support_point[i1] *= sign;
                        support_point_id |= 1 << i1;
                    }
                }
            }

            // Check edges.
            for i in 0..3 {
                let sign = local_dir[i].signum();

                // sign * local_dir[i] <= cos(pi / 2 - angle)
                if sign * local_dir[i] <= sang {
                    support_point[i] = -self.half_extents[i];
                    let p1 = Point::from_coordinates(support_point);
                    support_point[i] = self.half_extents[i];
                    let p2 = Point::from_coordinates(support_point);
                    out.push(
                        m * p1,
                        FeatureId::Vertex(support_point_id | (1 << i)),
                    );
                    out.push(
                        m * p2,
                        FeatureId::Vertex(support_point_id & !(1 << i)),
                    );

                    let edge_id = FeatureId::Edge(i | (support_point_id << 2));
                    out.push_edge_feature_id(edge_id);
                    out.set_feature_id(edge_id);
                    return;
                }
            }

            // We are not on a face or edge, return the support vertex.
            out.push(
                m * Point::from_coordinates(support_point),
                FeatureId::Vertex(support_point_id),
            );
            out.set_feature_id(FeatureId::Vertex(support_point_id));
        }
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId {
        let one_degree: N = na::convert(f64::consts::PI / 180.0);
        let cang = one_degree.cos();

        #[cfg(feature = "dim2")]
        {
            let mut support_point_id = 0;
            for i1 in 0..2 {
                let sign = local_dir[i1].signum();
                if sign * local_dir[i1] >= cang {
                    if sign > na::zero() {
                        return FeatureId::Face(i1);
                    } else {
                        return FeatureId::Face(i1 + 2);
                    }
                } else {
                    if sign < na::zero() {
                        support_point_id |= 1 << i1;
                    }
                }
            }

            // We are not on a face, return the support vertex.
            FeatureId::Vertex(support_point_id)
        }

        #[cfg(feature = "dim3")]
        {
            let sang = one_degree.sin();
            let mut support_point_id = 0;

            // Check faces.
            for i1 in 0..3 {
                let sign = local_dir[i1].signum();
                if sign * local_dir[i1] >= cang {
                    if sign > na::zero() {
                        return FeatureId::Face(i1);
                    } else {
                        return FeatureId::Face(i1 + 3);
                    }
                } else {
                    if sign < na::zero() {
                        support_point_id |= 1 << i1;
                    }
                }
            }

            // Check edges.
            for i in 0..3 {
                let sign = local_dir[i].signum();

                // sign * local_dir[i] <= cos(pi / 2 - angle)
                if sign * local_dir[i] <= sang {
                    return FeatureId::Edge(i | (support_point_id << 2));
                }
            }

            FeatureId::Vertex(support_point_id)
        }
    }

    #[cfg(feature = "dim2")]
    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<N> {
        match feature {
            FeatureId::Face(id) => {
                let mut dir: Vector<N> = na::zero();

                if id < 2 {
                    dir[id] = N::one();
                } else {
                    dir[id - 2] = -N::one();
                }
                return PolyhedralCone::HalfLine(Unit::new_unchecked(dir));
            }
            FeatureId::Vertex(id) => {
                let mut dir1: Vector<N> = na::zero();
                let mut dir2: Vector<N> = na::zero();

                match id {
                    0b00 => {
                        dir1[0] = N::one();
                        dir2[1] = N::one();
                    }
                    0b01 => {
                        dir1[1] = N::one();
                        dir2[0] = -N::one();
                    }
                    0b11 => {
                        dir1[0] = -N::one();
                        dir2[1] = -N::one();
                    }
                    0b10 => {
                        dir1[1] = -N::one();
                        dir2[0] = N::one();
                    }
                    _ => unreachable!(),
                }

                PolyhedralCone::Span([Unit::new_unchecked(dir1), Unit::new_unchecked(dir2)])
            }
            _ => panic!("Invalid feature ID {:?}.", feature)
        }
    }

    #[cfg(feature = "dim3")]
    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<N> {
        let mut generators = SmallVec::new();

        match feature {
            FeatureId::Face(id) => {
                let mut dir: Vector<N> = na::zero();

                if id < 3 {
                    dir[id] = N::one();
                } else {
                    dir[id - 3] = -N::one();
                }
                return PolyhedralCone::HalfLine(Unit::new_unchecked(dir));
            }
            FeatureId::Edge(id) => {
                let edge = id & 0b011;
                let face1 = (edge + 1) % 3;
                let face2 = (edge + 2) % 3;
                let signs = id >> 2;

                let mut dir1: Vector<N> = na::zero();
                let mut dir2: Vector<N> = na::zero();
                let _1: N = na::one();

                if signs & (1 << face1) != 0 {
                    dir1[face1] = -_1
                } else {
                    dir1[face1] = _1
                }

                if signs & (1 << face2) != 0 {
                    dir2[face2] = -_1
                } else {
                    dir2[face2] = _1;
                }

                generators.push(Unit::new_unchecked(dir1));
                generators.push(Unit::new_unchecked(dir2));
            }
            FeatureId::Vertex(id) => for i in 0..3 {
                let mut dir: Vector<N> = na::zero();
                let _1: N = na::one();

                if id & (1 << i) != 0 {
                    dir[i] = -_1;
                } else {
                    dir[i] = _1
                }

                generators.push(Unit::new_unchecked(dir));
            },
            _ => {}
        }

        PolyhedralCone::Span(generators)
    }
}
