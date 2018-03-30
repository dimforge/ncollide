//! Support mapping based Cuboid shape.

use std::f64;
use num::{One, Zero};

use na::{self, Real, Unit};
use bounding_volume::PolyhedralCone;
use shape::{ConvexPolyface, ConvexPolyhedron, FeatureId, SupportMap};
use math::{Isometry, Point, Vector};

/// Shape of a box.
#[derive(PartialEq, Debug, Clone)]
pub struct Cuboid<V> {
    half_extents: V,
}

// NOTE: format of the cuboid feature id:
//
// FeatureId::Vertex(id): the i-th bit of `id` is set to 1 iff. the i-th component of the vertex is negative.
// FeatureId::Edge(id): the part `id & 0b11` contains a number in [0,1] (or [0,2] in 3D) to indicate the axis (x, y, z).
//                      the part `id >> 2` follow the same rule as the vertex id.
// FeatureId::Face(id): if `id` lies in [0,1] (or [0 2] in 3D) indicates the axis (x, y, z) corresponding to the face normal.
//                      If `id` is greater than 1 (or 2 in 3D), then the negative axis (-x, -y, -z) is given by `id - 2` (or `id - 3` in 3D).
impl<V: Vector> Cuboid<V> {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be greater than 0.04.
    #[inline]
    pub fn new(half_extents: V) -> Cuboid<V> {
        for i in 0..na::dimension::<V>() {
            assert!(half_extents[i] >= V::Real::zero());
        }

        Cuboid {
            half_extents: half_extents,
        }
    }
}

impl<V> Cuboid<V> {
    /// The half-extents of this box. Half-extents are the box half-width along each axis.
    #[inline]
    pub fn half_extents(&self) -> &V {
        &self.half_extents
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for Cuboid<P::Vector> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_rotate_vector(dir);

        let mut res = *self.half_extents();

        for i in 0usize..na::dimension::<P::Vector>() {
            if local_dir[i] < P::Real::zero() {
                res[i] = -res[i];
            }
        }

        m.transform_point(&P::from_coordinates(res))
    }
}

impl<P: Point, M: Isometry<P>> ConvexPolyhedron<P, M> for Cuboid<P::Vector> {
    fn vertex(&self, id: FeatureId) -> P {
        let dim = na::dimension::<P::Vector>();
        let vid = id.vertex_id()
            .expect("The feature id does not identify a vertex.");
        let mut res = self.half_extents;

        for i in 0..dim {
            if vid & (1 << i) != 0 {
                res[i] = -res[i]
            }
        }

        P::from_coordinates(res)
    }

    fn edge(&self, id: FeatureId) -> (P, P, FeatureId, FeatureId) {
        let dim = na::dimension::<P::Vector>();
        let sub_id = id.subshape_id();
        let mut eid = id.edge_id()
            .expect("The feature id does not identify an edge.");

        let mut res = self.half_extents;

        if dim == 2 {
            let mut vertex_id = 0;
            if eid > 2 {
                eid -= 2;
                res[eid] = -res[eid];
                vertex_id |= 1 << eid;
            }

            let p1 = P::from_coordinates(res);
            let other_id = (eid + 1) % 2;
            res[other_id] = -res[other_id];
            let p2 = P::from_coordinates(res);
            let vid1 = FeatureId::vertex(sub_id, vertex_id);
            let vid2 = FeatureId::vertex(sub_id, vertex_id | (1 << other_id));

            (p1, p2, vid1, vid2)
        } else {
            let edge_i = eid & 0b11;
            let vertex_i = eid >> 2;

            for i in 0..dim {
                if i != edge_i && (vertex_i & (1 << i) != 0) {
                    res[i] = -res[i]
                }
            }

            let p1 = P::from_coordinates(res);
            res[edge_i] = -res[edge_i];
            let p2 = P::from_coordinates(res);
            let vid1 = FeatureId::vertex(sub_id, vertex_i & !(1 << edge_i));
            let vid2 = FeatureId::vertex(sub_id, vertex_i | (1 << edge_i));

            (p1, p2, vid1, vid2)
        }
    }

    fn face(&self, id: FeatureId, out: &mut ConvexPolyface<P>) {
        let mut i = id.face_id()
            .expect("The feature id does not identify a face.");
        let dim = na::dimension::<P::Vector>();
        let i1;
        let sign;

        if i < dim {
            i1 = i;
            sign = P::Real::one();
        } else {
            i1 = i - dim;
            sign = -P::Real::one();
        }

        if dim == 2 {
            let i2 = (i1 + 1) % 2;

            let mut vertex = self.half_extents;
            vertex[i1] *= sign;
            vertex[i2] *= if i1 == 0 { -sign } else { sign };

            let p1 = P::from_coordinates(vertex);
            vertex[i2] = -vertex[i2];
            let p2 = P::from_coordinates(vertex);

            let mut vertex_id1 = if sign < na::zero() { 1 << i1 } else { 0 };
            let mut vertex_id2 = vertex_id1;
            if p1[i2] < na::zero() {
                vertex_id1 |= 1 << i2;
            } else {
                vertex_id2 |= 1 << i2;
            }

            out.push(p1, FeatureId::vertex(0, vertex_id1));
            out.push(p2, FeatureId::vertex(0, vertex_id2));

            let mut normal: P::Vector = na::zero();
            normal[i1] = sign;
            out.set_normal(Unit::new_unchecked(normal));
            out.set_feature_id(FeatureId::edge(0, i));
        } else {
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
            out.push(P::from_coordinates(vertex), FeatureId::vertex(0, vertex_id));
            out.push_edge_feature_id(FeatureId::edge(0, edge_i2 | (vertex_id << 2)));

            vertex[i2] = -sign * self.half_extents[i2];
            vertex[i3] = sign * self.half_extents[i3];
            vertex_id |= msbit << i2 | sbit << i3;
            out.push(P::from_coordinates(vertex), FeatureId::vertex(0, vertex_id));
            out.push_edge_feature_id(FeatureId::edge(0, edge_i3 | (vertex_id << 2)));

            vertex[i2] = -self.half_extents[i2];
            vertex[i3] = -self.half_extents[i3];
            vertex_id |= 1 << i2 | 1 << i3;
            out.push(P::from_coordinates(vertex), FeatureId::vertex(0, vertex_id));
            out.push_edge_feature_id(FeatureId::edge(0, edge_i2 | (vertex_id << 2)));

            vertex[i2] = sign * self.half_extents[i2];
            vertex[i3] = -sign * self.half_extents[i3];
            vertex_id = sbit << i1 | sbit << i2 | msbit << i3;
            out.push(P::from_coordinates(vertex), FeatureId::vertex(0, vertex_id));
            out.push_edge_feature_id(FeatureId::edge(0, edge_i3 | (vertex_id << 2)));

            let mut normal: P::Vector = na::zero();
            normal[i1] = sign;
            out.set_normal(Unit::new_unchecked(normal));

            if sign > na::zero() {
                out.set_feature_id(FeatureId::face(0, i1));
            } else {
                out.set_feature_id(FeatureId::face(0, i1 + 3));
            }

            out.recompute_edge_normals_3d();
        }
    }

    fn support_face_toward(&self, m: &M, dir: &Unit<P::Vector>, out: &mut ConvexPolyface<P>) {
        out.clear();
        let local_dir = m.inverse_rotate_vector(dir);

        let mut iamax = 0;
        let mut amax = local_dir[0].abs();
        let dim = na::dimension::<P::Vector>();

        // FIXME: we should use nalgebra's iamax method.
        for i in 1..dim {
            let candidate = local_dir[i].abs();
            if candidate > amax {
                amax = candidate;
                iamax = i;
            }
        }

        if local_dir[iamax] > na::zero() {
            ConvexPolyhedron::<P, M>::face(self, FeatureId::face(0, iamax), out);
            out.transform_by(m);
        } else {
            ConvexPolyhedron::<P, M>::face(self, FeatureId::face(0, iamax + dim), out);
            out.transform_by(m);
        }
    }

    fn support_feature_toward(
        &self,
        m: &M,
        dir: &Unit<P::Vector>,
        angle: P::Real,
        out: &mut ConvexPolyface<P>,
    ) {
        let local_dir = m.inverse_rotate_vector(dir);
        let (sang, cang) = angle.sin_cos();
        let dim = na::dimension::<P::Vector>();
        let mut support_point = self.half_extents;

        out.clear();

        match dim {
            2 => {
                let mut support_point_id = 0;
                for i1 in 0..2 {
                    let sign = local_dir[i1].signum();
                    if sign * local_dir[i1] >= cang {
                        if sign > na::zero() {
                            ConvexPolyhedron::<P, M>::face(self, FeatureId::face(0, i1), out);
                            out.transform_by(m);
                        } else {
                            ConvexPolyhedron::<P, M>::face(self, FeatureId::face(0, i1 + 2), out);
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
                    m.transform_point(&P::from_coordinates(support_point)),
                    FeatureId::vertex(0, support_point_id),
                );
                out.set_feature_id(FeatureId::vertex(0, support_point_id));
            }
            3 => {
                let mut support_point_id = 0;

                // Check faces.
                for i1 in 0..3 {
                    let sign = local_dir[i1].signum();
                    if sign * local_dir[i1] >= cang {
                        if sign > na::zero() {
                            ConvexPolyhedron::<P, M>::face(self, FeatureId::face(0, i1), out);
                            out.transform_by(m);
                        } else {
                            ConvexPolyhedron::<P, M>::face(self, FeatureId::face(0, i1 + 3), out);
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
                        let p1 = P::from_coordinates(support_point);
                        support_point[i] = self.half_extents[i];
                        let p2 = P::from_coordinates(support_point);
                        out.push(
                            m.transform_point(&p1),
                            FeatureId::vertex(0, support_point_id | (1 << i)),
                        );
                        out.push(
                            m.transform_point(&p2),
                            FeatureId::vertex(0, support_point_id & !(1 << i)),
                        );

                        let edge_id = FeatureId::edge(0, i | (support_point_id << 2));
                        out.push_edge_feature_id(edge_id);
                        out.set_feature_id(edge_id);
                        return;
                    }
                }

                // We are not on a face or edge, return the support vertex.
                out.push(
                    m.transform_point(&P::from_coordinates(support_point)),
                    FeatureId::vertex(0, support_point_id),
                );
                out.set_feature_id(FeatureId::vertex(0, support_point_id));
            }
            _ => {
                out.push(self.support_point_toward(m, dir), FeatureId::Unknown);
                out.set_feature_id(FeatureId::Unknown);
            }
        }
    }

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<P> {
        let mut result = PolyhedralCone::new();

        match na::dimension::<P::Vector>() {
            2 => match feature {
                FeatureId::Edge { id, .. } => {
                    let mut dir: P::Vector = na::zero();

                    if id < 2 {
                        dir[id] = P::Real::one();
                    } else {
                        dir[id - 2] = -P::Real::one();
                    }
                    result.add_generator(Unit::new_unchecked(dir));
                }
                FeatureId::Vertex { id, .. } => {
                    let mut dir1: P::Vector = na::zero();
                    let mut dir2: P::Vector = na::zero();

                    match id {
                        0b00 => {
                            dir1[0] = P::Real::one();
                            dir2[1] = P::Real::one();
                        }
                        0b01 => {
                            dir1[1] = P::Real::one();
                            dir2[0] = -P::Real::one();
                        }
                        0b11 => {
                            dir1[0] = -P::Real::one();
                            dir2[1] = -P::Real::one();
                        }
                        0b10 => {
                            dir1[1] = -P::Real::one();
                            dir2[0] = P::Real::one();
                        }
                        _ => unreachable!(),
                    }

                    result.add_generator(Unit::new_unchecked(dir1));
                    result.add_generator(Unit::new_unchecked(dir2));
                }
                _ => {}
            },
            3 => match feature {
                FeatureId::Face { id, .. } => {
                    let mut dir: P::Vector = na::zero();

                    if id < 3 {
                        dir[id] = P::Real::one();
                    } else {
                        dir[id - 3] = -P::Real::one();
                    }
                    result.add_generator(Unit::new_unchecked(dir));
                }
                FeatureId::Edge { id, .. } => {
                    let edge = id & 0b011;
                    let face1 = (edge + 1) % 3;
                    let face2 = (edge + 2) % 3;
                    let signs = id >> 2;

                    let mut dir1: P::Vector = na::zero();
                    let mut dir2: P::Vector = na::zero();
                    let _1: P::Real = na::one();

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

                    result.add_generator(Unit::new_unchecked(dir1));
                    result.add_generator(Unit::new_unchecked(dir2));
                }
                FeatureId::Vertex { id, .. } => for i in 0..3 {
                    let mut dir: P::Vector = na::zero();
                    let _1: P::Real = na::one();

                    if id & (1 << i) != 0 {
                        dir[i] = -_1;
                    } else {
                        dir[i] = _1
                    }

                    result.add_generator(Unit::new_unchecked(dir));
                },
                _ => {}
            },
            _ => unimplemented!(),
        }

        result
    }
}
