use std::iter;
use na::{DVector, RealField, Point2};

use crate::bounding_volume::AABB;
use crate::query::{ContactPreprocessor, Contact, ContactKinematic};
use crate::shape::Segment;
use crate::math::Vector;


#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone, Debug)]
/// A 2D heightfield.
pub struct HeightField<N: RealField> {
    heights: DVector<N>,
    scale: Vector<N>,
    removed: Vec<bool>,
    aabb: AABB<N>,
}

impl<N: RealField> HeightField<N> {
    /// Creates a new 2D heightfield with the given heights and scale factor.
    pub fn new(heights: DVector<N>, scale: Vector<N>) -> Self {
        assert!(heights.len() > 1, "A heightfield heights must have at least 2 elements.");

        let max = heights.max();
        let min = heights.min();
        let hscale = scale * na::convert::<_, N>(0.5);
        let aabb = AABB::new(
            Point2::new(-hscale.x, min * scale.y),
            Point2::new(hscale.x, max * scale.y)
        );

        HeightField {
            heights, scale, aabb, removed: Vec::new()
        }
    }

    /// The number of cells of this heightfield.
    pub fn num_cells(&self) -> usize {
        self.heights.len() - 1
    }

    /// The height at each cell endpoint.
    pub fn heights(&self) -> &DVector<N> {
        &self.heights
    }

    /// The scale factor applied to this heightfield.
    pub fn scale(&self) -> &Vector<N> {
        &self.scale
    }

    /// The AABB of this heightfield.
    pub fn aabb(&self) -> &AABB<N> {
        &self.aabb
    }

    /// The width of a single cell of this heightfield.
    pub fn cell_width(&self) -> N {
        self.unit_cell_width() * self.scale.x
    }

    /// The width of a single cell of this heightfield, without taking the scale factor into account.
    pub fn unit_cell_width(&self) -> N {
        N::one() / na::convert(self.heights.len() as f64 - 1.0)
    }

    /// The left-most x-coordinate of this heightfield.
    pub fn start_x(&self) -> N {
        self.scale.x * na::convert(-0.5)
    }

    fn quantize_floor(&self, val: N, seg_length: N) -> usize {
        let _0_5: N = na::convert(0.5);
        let i = na::clamp(((val + _0_5) / seg_length).floor(), N::zero(), na::convert((self.num_cells() - 1) as f64));
        unsafe { na::convert_unchecked::<N, f64>(i) as usize }
    }

    fn quantize_ceil(&self, val: N, seg_length: N) -> usize {
        let _0_5: N = na::convert(0.5);
        let i = na::clamp(((val + _0_5) / seg_length).ceil(), N::zero(), na::convert(self.num_cells() as f64));
        unsafe { na::convert_unchecked::<N, f64>(i) as usize }
    }

    /// Index of the cell a point is on after vertical projection.
    pub fn cell_at_point(&self, pt: &Point2<N>) -> Option<usize> {
        let _0_5: N = na::convert(0.5);
        let scaled_pt = pt.coords.component_div(&self.scale);
        let seg_length = self.unit_cell_width();

        if scaled_pt.x < -_0_5 || scaled_pt.x > _0_5 {
            // Outside of the heightfield bounds.
            None
        } else {
            Some(self.quantize_floor(scaled_pt.x, seg_length))
        }
    }

    /// Iterator through all the segments of this heightfield.
    pub fn segments<'a>(&'a self) -> impl Iterator<Item = Segment<N>> + 'a {
        // FIXME: this is not very efficient since this wil
        // recompute shared points twice.
        (0..self.num_cells()).filter_map(move |i| self.segment_at(i))
    }

    /// The i-th segment of the heightfield if it has not been removed.
    pub fn segment_at(&self, i: usize) -> Option<Segment<N>> {
        if i >= self.num_cells() || self.is_segment_removed(i) {
            return None;
        }

        let _0_5: N = na::convert(0.5);
        let seg_length = N::one() / na::convert(self.heights.len() as f64 - 1.0);

        let x0 = -_0_5 + seg_length * na::convert(i as f64);
        let x1 = x0 + seg_length;

        let y0 = self.heights[i + 0];
        let y1 = self.heights[i + 1];

        let mut p0 = Point2::new(x0, y0);
        let mut p1 = Point2::new(x1, y1);

        // Apply scales:
        p0.coords.component_mul_assign(&self.scale);
        p1.coords.component_mul_assign(&self.scale);

        Some(Segment::new(p0, p1))
    }

    /// Mark the i-th segment of this heightfield as removed or not.
    pub fn set_segment_removed(&mut self, i: usize, removed: bool) {
        if self.removed.len() == 0 {
            self.removed = iter::repeat(false).take(self.num_cells()).collect()
        }

        self.removed[i] = removed
    }

    /// Checks if the i-th segment has been removed.
    pub fn is_segment_removed(&self, i: usize) -> bool {
        self.removed.len() != 0 && self.removed[i]
    }

    /// Applies `f` to each segment of this heightfield that intersects the given `aabb`.
    pub fn map_elements_in_local_aabb(&self, aabb: &AABB<N>, f: &mut impl FnMut(usize, &Segment<N>, &dyn ContactPreprocessor<N>)) {
        let _0_5: N = na::convert(0.5);
        let ref_mins = aabb.mins().coords.component_div(&self.scale);
        let ref_maxs = aabb.maxs().coords.component_div(&self.scale);
        let seg_length  = N::one() / na::convert(self.heights.len() as f64 - 1.0);

        if ref_maxs.x < -_0_5 || ref_mins.x > _0_5 {
            // Outside of the heightfield bounds.
            return;
        }

        let min_x = self.quantize_floor(ref_mins.x, seg_length);
        let max_x = self.quantize_ceil(ref_maxs.x, seg_length);

        // FIXME: find a way to avoid recomputing the same vertices
        // multiple times.
        for i in min_x..max_x {
            if self.is_segment_removed(i) {
                continue;
            }

            let x0 = -_0_5 +  seg_length * na::convert(i as f64);
            let x1 = x0 + seg_length;

            let y0 = self.heights[i + 0];
            let y1 = self.heights[i + 1];

            if (y0 > ref_maxs.y && y1 > ref_maxs.y) || (y0 < ref_mins.y && y1 < ref_mins.y) {
                continue;
            }

            let mut p0 = Point2::new(x0, y0);
            let mut p1 = Point2::new(x1, y1);

            // Apply scales:
            p0.coords.component_mul_assign(&self.scale);
            p1.coords.component_mul_assign(&self.scale);

            // Build the segment.
            let seg = Segment::new(p0, p1);

            // Build the contact preprocessor.
            let seg_id = i;
            let proc = HeightFieldTriangleContactPreprocessor::new(self, seg_id);

            // Call the callback.
            f(seg_id, &seg, &proc);
        }
    }
}


#[allow(dead_code)]
/// The contact preprocessor dedicated to 2D heightfields.
pub struct HeightFieldTriangleContactPreprocessor<'a, N: RealField> {
    heightfield: &'a HeightField<N>,
    triangle: usize
}

impl<'a, N: RealField> HeightFieldTriangleContactPreprocessor<'a, N> {
    /// Initialize a contact preprocessor for the given triangle of the given heightfield.
    pub fn new(heightfield: &'a HeightField<N>, triangle: usize) -> Self {
        HeightFieldTriangleContactPreprocessor {
            heightfield,
            triangle
        }
    }
}


impl<'a, N: RealField> ContactPreprocessor<N> for HeightFieldTriangleContactPreprocessor<'a, N> {
    fn process_contact(
        &self,
        _c: &mut Contact<N>,
        _kinematic: &mut ContactKinematic<N>,
        _is_first: bool)
        -> bool {
        /*
        // Fix the feature ID.
        let feature = if is_first {
            kinematic.feature1()
        } else {
            kinematic.feature2()
        };

        let face = &self.mesh.faces()[self.face_id];
        let actual_feature = match feature {
            FeatureId::Vertex(i) => FeatureId::Vertex(face.indices[i]),
            FeatureId::Edge(i) => FeatureId::Edge(face.edges[i]),
            FeatureId::Face(i) => {
                if i == 0 {
                    FeatureId::Face(self.face_id)
                } else {
                    FeatureId::Face(self.face_id + self.mesh.faces().len())
                }
            }
            FeatureId::Unknown => FeatureId::Unknown,
        };

        if is_first {
            kinematic.set_feature1(actual_feature);
        } else {
            kinematic.set_feature2(actual_feature);
        }

        // Test the validity of the LMD.
        if c.depth > N::zero() {
            true
        } else {
            // FIXME
        }
        */

        true
    }
}