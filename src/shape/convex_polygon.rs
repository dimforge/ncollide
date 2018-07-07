use std::f64;

use na::{self, Real, Unit};

use bounding_volume::PolyhedralCone;
use math::{Isometry, Point, Vector};
use shape::{ConvexPolygonalFeature, ConvexPolyhedron, FeatureId, SupportMap};
use transformation;
use utils::{self, IsometryOps};

/// A 2D convex polygon.
#[derive(Clone, Debug)]
pub struct ConvexPolygon<N: Real> {
    points: Vec<Point<N>>,
    normals: Vec<Unit<Vector<N>>>,
}

impl<N: Real> ConvexPolygon<N> {
    /// Creates a new 2D convex polygon from an arbitrary set of points.
    ///
    /// This explicitly computes the convex hull of the given set of points. Use
    /// Returns `None` if the convex hull computation failed.
    pub fn try_from_points(points: &[Point<N>]) -> Option<Self> {
        let hull = transformation::convex_hull(points);
        let mut vertices = hull.unwrap().0;
        vertices.reverse(); // FIXME: it is unfortunate to have to do this reverse.

        Self::try_new(vertices)
    }

    /// Creates a new 2D convex polygon from a set of points assumed to describe a convex polyline.
    ///
    /// Convexity of the input polyline is not checked.
    /// Returns `None` if some consecutive points are identical (or too close to being so).
    pub fn try_new(mut points: Vec<Point<N>>) -> Option<Self> {
        let eps = N::default_epsilon().sqrt();
        let mut normals = Vec::with_capacity(points.len());

        // First, compute all normals.
        for i1 in 0..points.len() {
            let i2 = (i1 + 1) % points.len();
            normals.push(utils::ccw_face_normal([&points[i1], &points[i2]])?);
        }

        let mut nremoved = 0;
        // See if the first vexrtex must be removed.
        if na::dot(&*normals[0], &*normals[normals.len() - 1]) > N::one() - eps {
            nremoved = 1;
        }

        // Second, find vertices that can be removed because
        // of collinearity of adjascent faces.
        for i2 in 1..points.len() {
            let i1 = i2 - 1;
            if na::dot(&*normals[i1], &*normals[i2]) > N::one() - eps {
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

    /// The vertices of this convex polygon.
    #[inline]
    pub fn points(&self) -> &[Point<N>] {
        &self.points
    }

    /// The normals of the edges of this convex polygon.
    #[inline]
    pub fn normals(&self) -> &[Unit<Vector<N>>] {
        &self.normals
    }
}

impl<N: Real> SupportMap<N> for ConvexPolygon<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        utils::point_cloud_support_point(dir, self.points())
    }
}

impl<N: Real> ConvexPolyhedron<N> for ConvexPolygon<N> {
    fn vertex(&self, id: FeatureId) -> Point<N> {
        self.points[id.unwrap_vertex()]
    }

    fn face(&self, id: FeatureId, out: &mut ConvexPolygonalFeature<N>) {
        out.clear();

        let ia = id.unwrap_face();
        let ib = (ia + 1) % self.points.len();
        out.push(self.points[ia], FeatureId::Vertex(ia));
        out.push(self.points[ib], FeatureId::Vertex(ib));

        out.set_normal(self.normals[ia]);
        out.set_feature_id(FeatureId::Face(ia));
    }

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<N> {
        match feature {
            FeatureId::Face(id) => PolyhedralCone::HalfLine(self.normals[id]),
            FeatureId::Vertex(id2) => {
                let id1 = if id2 == 0 {
                    self.normals.len() - 1
                } else {
                    id2 - 1
                };
                PolyhedralCone::Span([self.normals[id1], self.normals[id2]])
            }
            _ => unreachable!(),
        }
    }

    fn support_face_toward(
        &self,
        m: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        out: &mut ConvexPolygonalFeature<N>,
    ) {
        let ls_dir = m.inverse_transform_vector(dir);
        let mut best_face = 0;
        let mut max_dot = na::dot(&*self.normals[0], &ls_dir);

        for i in 1..self.points.len() {
            let dot = na::dot(&*self.normals[i], &ls_dir);

            if dot > max_dot {
                max_dot = dot;
                best_face = i;
            }
        }

        self.face(FeatureId::Face(best_face), out);
        out.transform_by(m);
    }

    fn support_feature_toward(
        &self,
        transform: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        _angle: N,
        out: &mut ConvexPolygonalFeature<N>,
    ) {
        out.clear();
        // FIXME: actualy find the support feature.
        self.support_face_toward(transform, dir, out)
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId {
        let eps: N = na::convert(f64::consts::PI / 180.0);
        let ceps = eps.cos();

        // Check faces.
        for i in 0..self.normals.len() {
            let normal = &self.normals[i];

            if na::dot(normal.as_ref(), local_dir.as_ref()) >= ceps {
                return FeatureId::Face(i);
            }
        }

        // Support vertex.
        FeatureId::Vertex(utils::point_cloud_support_point_id(
            local_dir.as_ref(),
            &self.points,
        ))
    }
}
