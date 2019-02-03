use crate::math::{Isometry, Point, Vector};
use na::{self, Real, Unit};
use crate::query::closest_points_internal;
use crate::query::Contact;
use crate::shape::{FeatureId, Shape};

/// A shape geometry type at the neighborhood of a point.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum NeighborhoodGeometry<N: Real> {
    /// A punctual approximation.
    Point,
    /// A line approximation.
    Line(Unit<Vector<N>>),
    /// A planar approximation.
    Plane(Unit<Vector<N>>),
}

/// The approximation of a shape on the neighborhood of a point.
#[derive(Clone, Debug)]
pub struct LocalShapeApproximation<N: Real> {
    // XXX: currently, there is no explicit representation
    // of the point where the approximation occurs in terms
    // of shape-specific parameters. That's because we work
    // so far with polyhedral approximations. Thus, it is
    // sufficient to known the feature alone to derive an
    // approximation.
    // In the future, we might want to:
    // - Use `parameters` as a set of shape-dependent coordinates giving the location of the
    //   point on it.
    // - Use `point` as the local-space point where the approximation occurs. It should be
    //   computed by the shape from the parameters.
    /// The shape feature the point lies on.
    pub feature: FeatureId,
    /// The point where approximation is computed.
    pub point: Point<N>,
    /// The approximation geometry.
    pub geometry: NeighborhoodGeometry<N>,
}

impl<N: Real> LocalShapeApproximation<N> {
    /// Initializes a new local shape approximation at `point`.
    pub fn new(feature: FeatureId, point: Point<N>, geometry: NeighborhoodGeometry<N>) -> Self {
        LocalShapeApproximation {
            feature,
            point,
            geometry,
        }
    }
}

/// Local contact kinematic of a pair of solids around two given points.
///
/// This is used to update the localization of contact points between two solids
/// from one frame to another. To achieve this, the local shape of the solids
/// around the given points are approximated by either dilated lines (unbounded
/// cylinders), planes, dilated points (spheres).
#[derive(Clone, Debug)]
pub struct ContactKinematic<N: Real> {
    approx1: LocalShapeApproximation<N>,
    approx2: LocalShapeApproximation<N>,

    margin1: N,
    margin2: N,
}

impl<N: Real> ContactKinematic<N> {
    /// Initializes an empty contact kinematic.
    ///
    /// All the contact kinematic information must be filled using methods
    /// prefixed by `set_`.
    pub fn new() -> Self {
        let approx = LocalShapeApproximation::new(
            FeatureId::Unknown,
            Point::origin(),
            NeighborhoodGeometry::Point,
        );

        ContactKinematic {
            margin1: na::zero(),
            margin2: na::zero(),
            approx1: approx.clone(),
            approx2: approx,
        }
    }

    /// Applies the given transformation to the first set of contact information.
    pub fn transform1(&mut self, m: &Isometry<N>) {
        self.approx1.point = m * self.approx1.point;

        match &mut self.approx1.geometry {
            NeighborhoodGeometry::Point => {}
            NeighborhoodGeometry::Plane(n) | NeighborhoodGeometry::Line(n) => {
                *n = m * &*n;
            }
        }
    }

    /// Applies the given transformation to the second set of contact information.
    pub fn transform2(&mut self, m: &Isometry<N>) {
        self.approx2.point = m * self.approx2.point;

        match &mut self.approx2.geometry {
            NeighborhoodGeometry::Point => {}
            NeighborhoodGeometry::Plane(n) | NeighborhoodGeometry::Line(n) => {
                *n = m * &*n;
            }
        }
    }

    /// The dilation of the first solid.
    pub fn dilation1(&self) -> N {
        self.margin1
    }

    /// The dilation of the second solid.
    pub fn dilation2(&self) -> N {
        self.margin2
    }

    /// The tracked point in local space of the first solid.
    ///
    /// This may not correspond to the contact point in the local
    /// space of the first since it does not takes the dilation
    /// into account.
    // FIXME: we might want to remove this in the future as it is not generalizable to surfaces.
    pub fn local1(&self) -> Point<N> {
        self.approx1.point
    }

    /// The tracked point in local space of the second solid.
    ///
    /// This may not correspond to the contact point in the local
    /// space of the second solid since it does not takes the dilation
    /// into account.
    // FIXME: we might want to remove this in the future as it is not generalizable to surfaces.
    pub fn local2(&self) -> Point<N> {
        self.approx2.point
    }

    /// The shape-dependent identifier of the feature of the first solid
    /// on which lies the contact point.
    pub fn feature1(&self) -> FeatureId {
        self.approx1.feature
    }

    /// The shape-dependent identifier of the feature of the second solid
    /// on which lies the contact point.
    pub fn feature2(&self) -> FeatureId {
        self.approx2.feature
    }

    /// Sets the shape-dependent identifier of the feature of the first solid
    /// on which lies the contact point.
    pub fn set_feature1(&mut self, f: FeatureId) {
        self.approx1.feature = f
    }

    /// Sets the shape-dependent identifier of the feature of the second solid
    /// on which lies the contact point.
    pub fn set_feature2(&mut self, f: FeatureId) {
        self.approx2.feature = f
    }

    /// Sets the dilation of the first solid.
    pub fn set_dilation1(&mut self, margin: N) {
        self.margin1 = margin;
    }

    /// Sets the dilation of the second solid.
    pub fn set_dilation2(&mut self, margin: N) {
        self.margin2 = margin;
    }

    /// The local approximation of the first shape.
    pub fn approx1(&self) -> &LocalShapeApproximation<N> {
        &self.approx1
    }

    /// The local approximation of the first shape.
    pub fn approx2(&self) -> &LocalShapeApproximation<N> {
        &self.approx2
    }

    /// The local approximation of the first shape.
    pub fn approx1_mut(&mut self) -> &mut LocalShapeApproximation<N> {
        &mut self.approx1
    }

    /// The local approximation of the second shape.
    pub fn approx2_mut(&mut self) -> &mut LocalShapeApproximation<N> {
        &mut self.approx2
    }

    /// Sets the local approximation of the first shape.
    pub fn set_approx1(
        &mut self,
        feature: FeatureId,
        point: Point<N>,
        geom: NeighborhoodGeometry<N>,
    )
    {
        self.approx1 = LocalShapeApproximation::new(feature, point, geom);
    }

    /// Sets the local approximation of the second shape.
    pub fn set_approx2(
        &mut self,
        feature: FeatureId,
        point: Point<N>,
        geom: NeighborhoodGeometry<N>,
    )
    {
        self.approx2 = LocalShapeApproximation::new(feature, point, geom);
    }

    /// Computes the updated contact points with the new positions of the solids.
    ///
    /// The vector `default_normal1` is the normal of the resulting contact
    /// in the rare case where the contact normal cannot be determined by the update.
    /// Typically, this should be set to the latest contact normal known.
    pub fn contact(
        &self,
        m1: &Isometry<N>,
        s1: &Shape<N>,
        deformations1: Option<&[N]>,
        m2: &Isometry<N>,
        s2: &Shape<N>,
        deformations2: Option<&[N]>,
        default_normal1: &Unit<Vector<N>>,
    ) -> Option<Contact<N>>
    {
        let normal;
        let mut depth;

        let mut world1 = m1 * self.approx1.point;
        let mut world2 = m2 * self.approx2.point;

        match (&self.approx1.geometry, &self.approx2.geometry) {
            (NeighborhoodGeometry::Plane(normal1), NeighborhoodGeometry::Point) => {
                normal = m1 * normal1;
                depth = -normal.dot(&(world2 - world1));
                world1 = world2 + *normal * depth;
            }
            (NeighborhoodGeometry::Point, NeighborhoodGeometry::Plane(normal2)) => {
                let world_normal2 = m2 * normal2;
                depth = -world_normal2.dot(&(world1 - world2));
                world2 = world1 + *world_normal2 * depth;
                normal = -world_normal2;
            }
            (NeighborhoodGeometry::Point, NeighborhoodGeometry::Point) => {
                if let Some((n, d)) = Unit::try_new_and_get(world2 - world1, N::default_epsilon()) {
                    if s1.tangent_cone_contains_dir(self.approx1.feature, m1, deformations1, &n)
                        || s2.tangent_cone_contains_dir(
                            self.approx2.feature,
                            m2,
                            deformations2,
                            &-n,
                        ) {
//                        println!("Is in tangent cone 3.");

                        depth = d;
                        normal = -n;
                    } else {
                        depth = -d;
                        normal = n;
                    }
                } else {
                    depth = na::zero();
                    normal = m1 * default_normal1;
                }
            }
            (NeighborhoodGeometry::Line(dir1), NeighborhoodGeometry::Point) => {
                let world_dir1 = m1 * dir1;
                let mut shift = world2 - world1;
                let proj = world_dir1.dot(&shift);
                shift -= dir1.into_inner() * proj;

                if let Some((n, d)) = Unit::try_new_and_get(shift, na::zero()) {
                    world1 = world2 + (-shift);

                    if s1.tangent_cone_contains_dir(self.approx1.feature, m1, deformations1, &n)
                        || s2.tangent_cone_contains_dir(
                            self.approx2.feature,
                            m2,
                            deformations2,
                            &-n,
                        ) {
//                        println!("Is in tangent cone 4.");

                        depth = d;
                        normal = -n;
                    } else {
                        depth = -d;
                        normal = n;
                    }
                } else {
                    depth = na::zero();
                    normal = m1 * default_normal1;
                }
            }
            (NeighborhoodGeometry::Point, NeighborhoodGeometry::Line(dir2)) => {
                let world_dir2 = m2 * dir2;
                let mut shift = world1 - world2;
                let proj = world_dir2.dot(&shift);
                shift -= dir2.into_inner() * proj;
                // NOTE: we set:
                // shift = world2 - world1
                let shift = -shift;

                if let Some((n, d)) = Unit::try_new_and_get(shift, na::zero()) {
                    world2 = world1 + shift;

                    if s1.tangent_cone_contains_dir(self.approx1.feature, m1, deformations1, &n)
                        || s2.tangent_cone_contains_dir(
                            self.approx2.feature,
                            m2,
                            deformations2,
                            &-n,
                        ) {
//                        println!("Is in tangent cone 5.");
                        depth = d;
                        normal = -n;
                    } else {
                        depth = -d;
                        normal = n;
                    }
                } else {
                    depth = na::zero();
                    normal = m1 * default_normal1;
                }
            }
            (NeighborhoodGeometry::Line(dir1), NeighborhoodGeometry::Line(dir2)) => {
                let world_dir1 = m1 * dir1;
                let world_dir2 = m2 * dir2;
                let (pt1, pt2) = closest_points_internal::line_against_line(
                    &world1,
                    &world_dir1,
                    &world2,
                    &world_dir2,
                );

                world1 = pt1;
                world2 = pt2;

                if let Some((n, d)) = Unit::try_new_and_get(world2 - world1, na::zero()) {
                    if s1.tangent_cone_contains_dir(self.approx1.feature, m1, deformations1, &n)
                        || s2.tangent_cone_contains_dir(
                            self.approx2.feature,
                            m2,
                            deformations2,
                            &-n,
                        ) {
//                        println!("Is in tangent cone 6.");

                        depth = d;
                        normal = -n;
                    } else {
                        depth = -d;
                        normal = n;
                    }
                } else {
                    depth = na::zero();
                    normal = m1 * default_normal1;
                }
            }
            _ => {
                return None;
            }
        }

//        println!("Before margin: {:?}", Contact::new(world1, world2, normal, depth));
        world1 += normal.into_inner() * self.margin1;
        world2 += normal.into_inner() * (-self.margin2);
        depth += self.margin1 + self.margin2;

        Some(Contact::new(world1, world2, normal, depth))
    }
}
