use std::collections::HashMap;
use na::{RealField};

use crate::query::{ContactKinematic, Contact, ContactPreprocessor, LocalShapeApproximation};
use crate::shape::{FeatureId, DeformableShape, DeformationsType};
use crate::bounding_volume::AABB;
use crate::math::{Point, Vector, DIM};

#[derive(PartialEq, Debug, Clone)]
struct HGrid<N: RealField> {
    cells: HashMap<Point<i64>, Vec<usize>>,
    cell_width: N,
}

impl<N: RealField> HGrid<N> {
    pub fn new(cell_width: N) -> Self {
        Self {
            cells: HashMap::new(),
            cell_width,
        }
    }

    fn quantify(value: N, cell_width: N) -> i64 {
        na::try_convert::<N, f64>((value / cell_width).floor()).unwrap() as i64
    }

    fn key(point: &Point<N>, cell_width: N) -> Point<i64> {
        Point::from(point.coords.map(|e| Self::quantify(e, cell_width)))
    }

    pub fn clear(&mut self) {
        self.cells.clear();
    }

    pub fn insert(&mut self, id: usize, point: &Point<N>) {
        let key = Self::key(point, self.cell_width);
        self.cells.entry(key).or_insert(Vec::new()).push(id)
    }

    pub fn elements_close_to_point<'a>(&'a self, point: &Point<N>, radius: N) -> impl Iterator<Item = usize> + 'a {
        let key = Self::key(point, self.cell_width);
        let quantified_radius = Self::quantify(radius, self.cell_width);
        let range = -(quantified_radius as i64)..=(quantified_radius as i64);
        let cells = &self.cells;

        NeighborCellsIterator::new(key, quantified_radius)
            .flat_map(move |cell| cells.get(&cell).into_iter())
            .flat_map(|cells| cells.iter())
            .cloned()
    }

    pub fn elements_intersecting_aabb<'a>(&'a self, aabb: &AABB<N>) -> impl Iterator<Item = usize> + 'a {
        self.cells.values().flat_map(|v| v).cloned()
    }

    pub fn elements_containing_point(&self, point: &Point<N>) -> impl Iterator<Item = usize> {
        std::iter::empty()
    }
}

struct NeighborCellsIterator {
    start: Point<i64>,
    end: Point<i64>,
    curr: Point<i64>,
    done: bool,
}

impl NeighborCellsIterator {
    fn new(center: Point<i64>, radius: i64) -> Self {
        let start = center - Vector::repeat(radius as i64);
        Self {
            start,
            end: center + Vector::repeat(radius as i64),
            curr: start,
            done: false,
        }
    }
}

impl Iterator for NeighborCellsIterator {
    type Item = Point<i64>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }

        if self.curr == self.end {
            self.done = true;
            Some(self.curr)
        } else {
            let result = self.curr;

            for i in 0..DIM {
                self.curr[i] += 1;

                if self.curr[i] > self.end[i] {
                    self.curr[i] = self.start[i];
                } else {
                    break;
                }
            }

            Some(result)
        }
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
            grid.insert(i, c)
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

    /// An iterator over a conservative list of indices of the balls that are potentially closer than a distance of `radius`.
    ///
    /// The return list of indices is conservative, meaning that it will return all the balls at a radius smaller or
    /// equal to `radius`, but it may also return other balls.
    pub fn balls_close_to_point<'a>(&'a self, point: &Point<N>, radius: N) -> impl Iterator<Item = usize> + 'a {
        self.grid.elements_close_to_point(point, radius)
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


impl<N: RealField> DeformableShape<N> for Multiball<N> {
    fn deformations_type(&self) -> DeformationsType {
        DeformationsType::Vectors
    }

    /// Updates all the degrees of freedom of this shape.
    fn set_deformations(&mut self, coords: &[N]) {
        assert!(coords.len() >= self.centers.len() * DIM, "Set deformations error: dimension mismatch.");

        // There is a bit of unsafe code in order to perform a memcopy for
        // efficiency reasons when the mapping between degrees of freedom
        // is trivial.
        unsafe {
            let len = self.centers.len();
            let coords_ptr = coords.as_ptr() as *const Point<N>;
            let coords_pt: &[Point<N>] = std::slice::from_raw_parts(coords_ptr, len);
            self.centers.copy_from_slice(coords_pt);
        }

        // XXX: perform an update more efficient than
        // recomputing the whole grid.
        self.grid.clear();

        for (i, c) in self.centers.iter().enumerate() {
            self.grid.insert(i, c)
        }
    }

    fn update_local_approximation(
        &self,
        coords: &[N],
        approx: &mut LocalShapeApproximation<N>,
    )
    {
        match approx.feature {
            FeatureId::Face(i) => {
                let i_first_coord = i * DIM;
                approx.point.coords.copy_from_slice(&coords[i_first_coord..i_first_coord + DIM]);
            },
            _ => panic!(
                "Encountered invalid multiball feature: {:?}. Only FeatureId::Face are possible",
                approx.feature
            ),
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