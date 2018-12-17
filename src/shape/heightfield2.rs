use na::{DVector, Real, Point2};

use crate::bounding_volume::AABB;
use crate::query::{ContactPreprocessor, Contact, ContactKinematic};
use crate::shape::Segment;
use crate::math::Vector;


#[derive(Clone, Debug)]
pub struct HeightField<N: Real> {
    heights: DVector<N>,
    scale: Vector<N>,
    aabb: AABB<N>,
    num_segments: usize
}

impl<N: Real> HeightField<N> {
    pub fn new(heights: DVector<N>, scale: Vector<N>) -> Self {
        let max = heights.max();
        let min = heights.min();
        let hscale = scale * na::convert::<_, N>(0.5);
        let aabb = AABB::new(
            Point2::new(-hscale.x, min * scale.y),
            Point2::new(hscale.x, max * scale.y)
        );
        let num_segments = heights.len() - 1;

        HeightField {
            heights, scale, aabb, num_segments
        }
    }

    pub fn heights(&self) -> &DVector<N> {
        &self.heights
    }

    pub fn scale(&self) -> &Vector<N> {
        &self.scale
    }

    pub fn aabb(&self) -> &AABB<N> {
        &self.aabb
    }

    pub fn map_elements_in_local_aabb(&self, aabb: &AABB<N>, f: &mut impl FnMut(usize, &Segment<N>, &ContactPreprocessor<N>)) {
        let _0_5: N = na::convert(0.5);
        let ref_mins = aabb.mins().coords.component_div(&self.scale);
        let ref_maxs = aabb.maxs().coords.component_div(&self.scale);
        let seg_length  = N::one() / na::convert(self.heights.len() as f64 - 1.0);

        if ref_maxs.x <= -_0_5 || ref_mins.x >= _0_5 {
            // Outside of the heightfield bounds.
            return;
        }

        let min_x = unsafe { na::convert_unchecked::<N, f64>((na::clamp(ref_mins.x + _0_5, N::zero(), N::one()) / seg_length).floor()) } as usize;
        let max_x = unsafe { na::convert_unchecked::<N, f64>((na::clamp(ref_maxs.x + _0_5, N::zero(), N::one()) / seg_length).ceil()) } as usize;

        // FIXME: find a way to avoid recomputing the same vertices
        // multiple times.
        for i in min_x..max_x {
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
            p0.coords.component_mul_mut(&self.scale);
            p1.coords.component_mul_mut(&self.scale);

            // Build the two triangles.
            let seg = Segment::new(p0, p1);

            // Build the contact preprocessors.
            let seg_id = i;
            let proc = HeightFieldTriangleContactPreprocessor::new(self, seg_id);

            // Call the callback.
            f(seg_id, &seg, &proc);
        }
    }
}



pub struct HeightFieldTriangleContactPreprocessor<'a, N: Real> {
    heightfield: &'a HeightField<N>,
    triangle: usize
}

impl<'a, N: Real> HeightFieldTriangleContactPreprocessor<'a, N> {
    pub fn new(heightfield: &'a HeightField<N>, triangle: usize) -> Self {
        HeightFieldTriangleContactPreprocessor {
            heightfield,
            triangle
        }
    }
}


impl<'a, N: Real> ContactPreprocessor<N> for HeightFieldTriangleContactPreprocessor<'a, N> {
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool)
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