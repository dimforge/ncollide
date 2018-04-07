use std::collections::HashMap;
use std::collections::hash_map::Entry;
use num::One;
use approx::ApproxEq;

use na::{self, Real, Unit};

use utils::{self, data::SortedPair};
use bounding_volume::PolyhedralCone;
use shape::{ConvexPolyface, ConvexPolyhedron, FeatureId, SupportMap};
use math::{Isometry, Point, Vector};

pub struct ConvexPolygon<P: Point> {
    points: Vec<P>,
    normals: Vec<Unit<P::Vector>>,
}

impl<P: Point> ConvexPolygon<P> {
    pub fn try_new(mut points: Vec<P>) -> Option<Self> {
        let eps = P::Real::default_epsilon().sqrt();
        let mut normals = Vec::with_capacity(points.len());

        // First, compute all normals.
        for i1 in 0..points.len() {
            let i2 = (i1 + 1) % points.len();
            normals.push(P::ccw_face_normal(&[&points[i1], &points[i2]])?);
        }

        let mut nremoved = 0;
        // See if th efirst vexrtex must be removed.
        if na::dot(&*normals[0], &*normals[normals.len() - 1]) > P::Real::one() - eps {
            nremoved = 1;
        }

        // Second, find vertices that can be removed because
        // of collinearity of adjascent faces.
        for i2 in 1..points.len() {
            let i1 = i2 - 1;
            if na::dot(&*normals[i1], &*normals[i2]) > P::Real::one() - eps {
                // Remove
                nremoved += 1;
            } else {
                points[i2 - nremoved] = points[i2];
                normals[i2 - nremoved] = normals[i2];
            }
        }

        let new_length = points.len() - nremoved;
        points.truncate(new_length);
        normals.truncate(new_length);

        if points.len() != 0 {
            Some(ConvexPolygon { points, normals })
        } else {
            None
        }
    }

    #[inline]
    pub fn points(&self) -> &[P] {
        &self.points
    }

    #[inline]
    pub fn normals(&self) -> &[Unit<P::Vector>] {
        &self.normals
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for ConvexPolygon<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_rotate_vector(dir);
        let best_pt = utils::point_cloud_support_point(&local_dir, self.points());

        m.transform_point(&best_pt)
    }
}

impl<P: Point, M: Isometry<P>> ConvexPolyhedron<P, M> for ConvexPolygon<P> {
    fn vertex(&self, id: FeatureId) -> P {
        self.points[id.unwrap_vertex()]
    }

    fn edge(&self, id: FeatureId) -> (P, P, FeatureId, FeatureId) {
        let ia = id.unwrap_edge();
        let ib = (ia + 1) % self.points.len();

        (
            self.points[ia],
            self.points[ib],
            FeatureId::Vertex(ia),
            FeatureId::Vertex(ib),
        )
    }

    fn face(&self, id: FeatureId, out: &mut ConvexPolyface<P>) {
        let dim = na::dimension::<P::Vector>();
        out.clear();

        if dim == 2 {
            let ia = id.unwrap_face();
            let ib = (ia + 1) % self.points.len();
            out.push(self.points[ia], FeatureId::Vertex(ia));
            out.push(self.points[ib], FeatureId::Vertex(ib));
            out.push_edge_feature_id(FeatureId::Edge(ia));

            out.set_normal(self.normals[ia]);
            out.set_feature_id(FeatureId::Edge(ia));
        } else {
            unimplemented!()
        }
    }

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<P> {
        let dim = na::dimension::<P::Vector>();
        let mut polycone = PolyhedralCone::new();

        if dim == 2 {
            match feature {
                FeatureId::Edge(id) => polycone.add_generator(self.normals[id]),
                FeatureId::Vertex(id1) => {
                    let id2 = (id1 + 1) % self.normals.len();
                    polycone.add_generator(self.normals[id1]);
                    polycone.add_generator(self.normals[id2]);
                }
                _ => unreachable!(),
            }
        } else {
            unimplemented!()
        }

        polycone
    }

    fn support_face_toward(&self, m: &M, dir: &Unit<P::Vector>, out: &mut ConvexPolyface<P>) {
        let ls_dir = m.inverse_rotate_vector(dir);
        let mut best_face = 0;
        let mut max_dot = na::dot(&*self.normals[0], &ls_dir);

        for i in 1..self.points.len() {
            let dot = na::dot(&*self.normals[i], &ls_dir);

            if dot > max_dot {
                max_dot = dot;
                best_face = i;
            }
        }

        ConvexPolyhedron::<P, M>::face(self, FeatureId::Face(best_face), out);
        out.transform_by(m);
    }

    fn support_feature_toward(
        &self,
        transform: &M,
        dir: &Unit<P::Vector>,
        _angle: P::Real,
        out: &mut ConvexPolyface<P>,
    ) {
        out.clear();
        // FIXME: actualy find the support feature.
        self.support_face_toward(transform, dir, out)
    }
}
