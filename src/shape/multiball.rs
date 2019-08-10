use std::collections::HashMap;
use na::{RealField};

use crate::query::{ContactKinematic, Contact, ContactPreprocessor};
use crate::shape::FeatureId;
use crate::bounding_volume::AABB;
use crate::math::Point;

#[derive(PartialEq, Debug, Clone)]
struct HGrid<N: RealField> {
    cells: HashMap<Point<u64>, Vec<usize>>,
    cell_width: N,
}

impl<N: RealField> HGrid<N> {
    pub fn new(cell_width: N) -> Self {
        Self {
            cells: HashMap::new(),
            cell_width,
        }
    }

    pub fn insert(&mut self, id: usize, point: &Point<N>, half_extent: N) {
        // XXX
        self.cells.entry(Point::origin()).or_insert(Vec::new()).push(id)
    }

    pub fn elements_intersecting_aabb<'a>(&'a self, aabb: &AABB<N>) -> impl Iterator<Item = usize> + 'a {
        self.cells.values().flat_map(|v| v).cloned()
    }

    pub fn elements_containing_point(&self, point: &Point<N>) -> impl Iterator<Item = usize> {
        std::iter::empty()
    }
}

/// A shape composed of a set of balls with the same radius.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(PartialEq, Debug, Clone)]
pub struct Multiball<N: RealField> {
    radius: N,
    centers: Vec<Point<N>>,
    grid: HGrid<N>
}

impl<N: RealField> Multiball<N> {
    /// Initializes a set fo balls with the same radius.
    pub fn new(radius: N, centers: Vec<Point<N>>) -> Self {
        let cell_width = radius * na::convert(3.0);
        let mut grid = HGrid::new(cell_width);

        for (i, c) in centers.iter().enumerate() {
            grid.insert(i, c, radius)
        }

        Self {
            radius, centers, grid
        }
    }

    /// The radius of all balls on this sets.
    pub fn radius(&self) -> N {
        self.radius
    }

    /// The center of each boll on this set.
    pub fn centers(&self) -> &[Point<N>] {
        &self.centers
    }


    /// An iterator over the indices of the balls that may intersect the given aabb.
    pub fn potential_balls_intersecting_aabb<'a>(&'a self, aabb: &AABB<N>) -> impl Iterator<Item = usize> + 'a {
        self.grid.elements_intersecting_aabb(aabb)
    }

    /// An iterator over the indices of the balls that contain the given point.
    pub fn balls_containing_point(&self, point: &Point<N>) -> impl Iterator<Item = usize> {
        self.grid.elements_containing_point(point)
    }

    /// The contact preprocessor to be used for contact determination with the given ball of this multiball.
    #[inline]
    pub fn contact_preprocessor(&self, i: usize) -> impl ContactPreprocessor<N> {
        MultiballContactPreprocessor {
            ball_id: i,
            center: self.centers[i],
        }
    }

    /// Given a feature-id of this multiball, returns the index and feature-id of the corresponding ball.
    pub fn subshape_feature_id(&self, fid: FeatureId) -> (usize, FeatureId) {
        match fid {
            FeatureId::Face(i) => (i, FeatureId::Face(0)),
            #[cfg(feature = "dim3")]
            FeatureId::Edge(i) => (i, FeatureId::Edge(0)),
            FeatureId::Vertex(i) => (i, FeatureId::Vertex(0)),
            FeatureId::Unknown => (0, FeatureId::Unknown)
        }
    }
}


struct MultiballContactPreprocessor<N: RealField> {
    ball_id: usize,
    center: Point<N>,
}

impl<N: RealField> ContactPreprocessor<N> for MultiballContactPreprocessor<N> {
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool)
        -> bool {

        // Fix the feature ID.
        let feature = if is_first {
            kinematic.feature1()
        } else {
            kinematic.feature2()
        };

        let actual_feature = match feature {
            FeatureId::Face(_) => FeatureId::Face(self.ball_id),
            _ => return false,
        };

        if is_first {
            kinematic.set_feature1(actual_feature);
            kinematic.translate1(&self.center.coords.into());
        } else {
            kinematic.set_feature2(actual_feature);
            kinematic.translate2(&self.center.coords.into());
        }

        true
    }
}