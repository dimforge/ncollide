//! Support mapping based Cuboid shape.

use num::Zero;
use approx::ApproxEq;

use na::{self, Real, Unit};
use shape::{ConvexPolyface, FeatureId, SupportMap};
use math::{Isometry, Point, Vector};

/// Shape of a box.
#[derive(PartialEq, Debug, Clone)]
pub struct Cuboid<V> {
    half_extents: V,
}

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

    #[inline]
    fn support_area_toward(
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
                        out.push(m.transform_point(&p1), FeatureId::Vertex(vertex_id1));
                        out.push(m.transform_point(&p2), FeatureId::Vertex(vertex_id2));

                        let mut normal: P::Vector = na::zero();
                        normal[i1] = sign;
                        out.set_normal(Unit::new_unchecked(m.transform_vector(&normal)));

                        if sign > na::zero() {
                            out.set_feature_id(FeatureId::Edge(i1));
                        } else {
                            out.set_feature_id(FeatureId::Edge(i1 + 2));
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
                    FeatureId::Vertex(support_point_id),
                );
                out.set_feature_id(FeatureId::Vertex(support_point_id));
            }
            3 => {
                // Check faces.
                for i1 in 0..3 {
                    let sign = local_dir[i1].signum();
                    if sign * local_dir[i1] >= cang {
                        let i2 = (i1 + 1) % 3;
                        let i3 = (i1 + 2) % 3;
                        let mut vertex = self.half_extents;
                        vertex[i1] *= sign;

                        let (sbit, msbit) = if sign < na::zero() { (1, 0) } else { (0, 1) };
                        let mut vertex_id = sbit << i1;
                        let mut edge_id = i1;
                        out.push(
                            m.transform_point(&P::from_coordinates(vertex)),
                            FeatureId::Vertex(vertex_id),
                        );

                        vertex[i2] = -sign * self.half_extents[i2];
                        vertex[i3] = sign * self.half_extents[i3];
                        vertex_id |= msbit << i2 | sbit << i3;
                        out.push(
                            m.transform_point(&P::from_coordinates(vertex)),
                            FeatureId::Vertex(vertex_id),
                        );

                        vertex[i2] = -self.half_extents[i2];
                        vertex[i3] = -self.half_extents[i3];
                        vertex_id |= 1 << i2 | 1 << i3;
                        out.push(
                            m.transform_point(&P::from_coordinates(vertex)),
                            FeatureId::Vertex(vertex_id),
                        );

                        vertex[i2] = sign * self.half_extents[i2];
                        vertex[i3] = -sign * self.half_extents[i3];
                        vertex_id = sbit << i1 | sbit << i2 | msbit << i3;
                        out.push(
                            m.transform_point(&P::from_coordinates(vertex)),
                            FeatureId::Vertex(vertex_id),
                        );

                        let mut normal: P::Vector = na::zero();
                        normal[i1] = sign;
                        out.set_normal(Unit::new_unchecked(m.transform_vector(&normal)));
                        if sign > na::zero() {
                            out.set_feature_id(FeatureId::Face(i1));
                        } else {
                            out.set_feature_id(FeatureId::Face(i1 + 3));
                        }
                        out.recompute_edge_normals_3d();
                        return;
                    } else {
                        support_point[i1] *= sign;
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
                        out.push(m.transform_point(&p1), FeatureId::Vertex(0 /* FIXME */));
                        out.push(m.transform_point(&p2), FeatureId::Vertex(0 /* FIXME */));
                        return;
                    }
                }

                // We are not on a face or edge, return the support vertex.
                out.push(
                    m.transform_point(&P::from_coordinates(support_point)),
                    FeatureId::Vertex(0 /* FIXME */),
                );
            }
            _ => out.push(
                self.support_point_toward(m, dir),
                FeatureId::Vertex(0 /* FIXME */),
            ),
        }
    }

    fn is_direction_in_normal_cone(
        &self,
        m: &M,
        feature: FeatureId,
        point: &P,
        dir: &Unit<P::Vector>,
    ) -> bool {
        let tolerence: P::Real = na::convert(1.0 / 180.0);
        let (stol, ctol) = tolerence.sin_cos();
        let mut local_dir = m.inverse_rotate_vector(dir);

        match na::dimension::<P::Vector>() {
            2 => match feature {
                FeatureId::Edge(id) => {
                    if id < 2 {
                        local_dir[id] >= ctol
                    } else {
                        -local_dir[id - 2] >= ctol
                    }
                }
                FeatureId::Vertex(id) => {
                    if id & 0b01 != 0 {
                        local_dir[0] = -local_dir[0]
                    }
                    if id & 0b10 != 0 {
                        local_dir[1] = -local_dir[1]
                    }

                    local_dir[0] >= -stol && local_dir[1] >= -stol
                }
                _ => panic!("A 2D shape does not have a feature other than edge or vertex."),
            },
            3 => match feature {
                FeatureId::Face(id) => false,
                FeatureId::Edge(id) => false,
                FeatureId::Vertex(id) => false,
            },
            _ => false,
        }
    }
}
