use bounding_volume::PolyhedralCone;
use math::{Isometry, Point, Vector};
use na::{self, Real, Unit};
use query::closest_points_internal;
use query::Contact;
use shape::FeatureId;
use utils::IsometryOps;

/// Approximation of a shape at the neighborhood of a point.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LocalShapeApproximation<N: Real> {
    /// A punctual approximation.
    Point(Point<N>),
    /// A line approximation.
    Line(Point<N>, Unit<Vector<N>>),
    /// A planar approximation.
    Plane(Point<N>, Unit<Vector<N>>),
}

/// Local contact kinematic of a pair of solids around two given points.
/// 
/// This is used to update the localization of contact points between two solids
/// from one frame to another. To achieve this, the local shape of the solids
/// around the given points are approximated by either dilated lines (unbounded
/// cylinders), planes, dilated points (spheres).
#[derive(Clone, Debug)]
pub struct ContactKinematic<N: Real> {
    normals1: PolyhedralCone<N>,
    normals2: PolyhedralCone<N>,

    margin1: N,
    margin2: N,

    feature1: FeatureId,
    feature2: FeatureId,

    approx1: LocalShapeApproximation<N>,
    approx2: LocalShapeApproximation<N>,
}

impl<N: Real> ContactKinematic<N> {
    /// Initializes an empty contact kinematic.
    /// 
    /// All the contact kinematic information must be filled using methods
    /// prefixed by `set_`.
    pub fn new() -> Self {
        ContactKinematic {
            margin1: na::zero(),
            margin2: na::zero(),
            normals1: PolyhedralCone::Full,
            normals2: PolyhedralCone::Full,
            feature1: FeatureId::Unknown,
            feature2: FeatureId::Unknown,
            approx1: LocalShapeApproximation::Point(Point::origin()),
            approx2: LocalShapeApproximation::Point(Point::origin()),
        }
    }

    /// Applies the given transformation to the first set of contact information.
    pub fn transform1(&mut self, m: &Isometry<N>) {
        self.normals1.transform_by(m);

        match &mut self.approx1 {
            LocalShapeApproximation::Point(pt) => *pt = m * &*pt,
            LocalShapeApproximation::Plane(pt, n) | LocalShapeApproximation::Line(pt, n) => {
                *pt = m * &*pt;
                *n = m * &*n;
            }
        }
    }

    /// Applies the given transformation to the second set of contact information.
    pub fn transform2(&mut self, m: &Isometry<N>) {
        self.normals2.transform_by(m);

        match &mut self.approx2 {
            LocalShapeApproximation::Point(pt) => *pt = m * &*pt,
            LocalShapeApproximation::Plane(pt, n) | LocalShapeApproximation::Line(pt, n) => {
                *pt = m * &*pt;
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
        match self.approx1 {
            LocalShapeApproximation::Point(pt) |
            LocalShapeApproximation::Plane(pt, _) |
            LocalShapeApproximation::Line(pt, _) => pt
        }
    }

    /// The tracked point in local space of the second solid.
    ///
    /// This may not correspond to the contact point in the local
    /// space of the second solid since it does not takes the dilation
    /// into account.
    // FIXME: we might want to remove this in the future as it is not generalizable to surfaces.
    pub fn local2(&self) -> Point<N> {
        match self.approx2 {
            LocalShapeApproximation::Point(pt) |
            LocalShapeApproximation::Plane(pt, _) |
            LocalShapeApproximation::Line(pt, _) => pt
        }
    }

    /// The shape-dependent identifier of the feature of the first solid
    /// on which lies the contact point.
    pub fn feature1(&self) -> FeatureId {
        self.feature1
    }

    /// The shape-dependent identifier of the feature of the second solid
    /// on which lies the contact point.
    pub fn feature2(&self) -> FeatureId {
        self.feature2
    }

    /// Sets the dilation of the first solid.
    pub fn set_dilation1(&mut self, margin: N) {
        self.margin1 = margin;
    }

    /// Sets the dilation of the second solid.
    pub fn set_dilation2(&mut self, margin: N) {
        self.margin2 = margin;
    }

    /// Sets the local approximation of the first shape.
    pub fn set_approx1(&mut self, fid: FeatureId, approx: LocalShapeApproximation<N>, normals: PolyhedralCone<N>) {
        self.feature1 = fid;
        self.approx1 = approx;
        self.normals1 = normals;
    }

    /// Sets the local approximation of the second shape.
    pub fn set_approx2(&mut self, fid: FeatureId, approx: LocalShapeApproximation<N>, normals: PolyhedralCone<N>) {
        self.feature2 = fid;
        self.approx2 = approx;
        self.normals2 = normals;
    }


    /// Computes the updated contact points with the new positions of the solids.
    ///
    /// The vector `default_normal1` is the normal of the resulting contactc
    /// in the rare case where the contact normal cannot be determined by the update.
    /// Typically, this should be set to the latest contact normal known.
    pub fn contact(
        &self,
        m1: &Isometry<N>,
        m2: &Isometry<N>,
        default_normal1: &Unit<Vector<N>>,
    ) -> Option<Contact<N>> {
        let mut world1;
        let mut world2;
        let normal;
        let mut depth;

        match (&self.approx1, &self.approx2) {
            (LocalShapeApproximation::Plane(local1, _), LocalShapeApproximation::Point(local2)) => {
                world1 = m1 * local1;
                world2 = m2 * local2;
                normal = m1 * self.normals1.unwrap_half_line();
                depth = -na::dot(normal.as_ref(), &(world2 - world1));
                world1 = world2 + *normal * depth;
            }
            (LocalShapeApproximation::Point(local1), LocalShapeApproximation::Plane(local2, _)) => {
                world1 = m1 * local1;
                world2 = m2 * local2;
                let world_normal2 = m2 * self.normals2.unwrap_half_line();
                depth = -na::dot(&*world_normal2, &(world1 - world2));
                world2 = world1 + *world_normal2 * depth;
                normal = -world_normal2;
            }
            (LocalShapeApproximation::Point(local1), LocalShapeApproximation::Point(local2)) => {
                world1 = m1 * local1;
                world2 = m2 * local2;

                if let Some((n, d)) =
                Unit::try_new_and_get(world2 - world1, N::default_epsilon())
                    {
                        let local_n1 = m1.inverse_transform_unit_vector(&n);
                        let local_n2 = m2.inverse_transform_unit_vector(&-n);
                        if self.normals1.polar_contains_dir(&local_n1)
                            || self.normals2.polar_contains_dir(&local_n2)
                            {
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
            (LocalShapeApproximation::Line(local1, dir1), LocalShapeApproximation::Point(local2)) => {
                world1 = m1 * local1;
                world2 = m2 * local2;

                let world_dir1 = m1 * dir1;
                let mut shift = world2 - world1;
                let proj = na::dot(world_dir1.as_ref(), &shift);
                shift -= dir1.unwrap() * proj;

                if let Some((n, d)) = Unit::try_new_and_get(shift, na::zero()) {
                    let local_n1 = m1.inverse_transform_unit_vector(&n);
                    let local_n2 = m2.inverse_transform_unit_vector(&-n);
                    world1 = world2 + (-shift);

                    if self.normals1.polar_contains_dir(&local_n1) || self.normals2.polar_contains_dir(&local_n2)
                        {
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
            (LocalShapeApproximation::Point(local1), LocalShapeApproximation::Line(local2, dir2)) => {
                world1 = m1 * local1;
                world2 = m2 * local2;

                let world_dir2 = m2 * dir2;
                let mut shift = world1 - world2;
                let proj = na::dot(world_dir2.as_ref(), &shift);
                shift -= dir2.unwrap() * proj;
                // NOTE: we set:
                // shift = world2 - world1
                let shift = -shift;

                if let Some((n, d)) = Unit::try_new_and_get(shift, na::zero()) {
                    let local_n1 = m1.inverse_transform_unit_vector(&n);
                    let local_n2 = m2.inverse_transform_unit_vector(&-n);
                    world2 = world1 + shift;

                    if self.normals1.polar_contains_dir(&local_n1)
                        || self.normals2.polar_contains_dir(&local_n2)
                        {
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
            (LocalShapeApproximation::Line(local1, dir1), LocalShapeApproximation::Line(local2, dir2)) => {
                world1 = m1 * local1;
                world2 = m2 * local2;

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
                    let local_n1 = m1.inverse_transform_unit_vector(&n);
                    let local_n2 = m2.inverse_transform_unit_vector(&-n);

                    if self.normals1.polar_contains_dir(&local_n1)
                        || self.normals2.polar_contains_dir(&local_n2)
                        {
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

        world1 += normal.unwrap() * self.margin1;
        world2 += normal.unwrap() * (-self.margin2);
        depth += self.margin1 + self.margin2;

        Some(Contact::new(world1, world2, normal, depth))
    }
}
