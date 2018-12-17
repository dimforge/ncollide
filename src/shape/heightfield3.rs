use na::{DMatrix, Real, Point3};

use crate::bounding_volume::AABB;
use crate::query::{ContactPreprocessor, Contact, ContactKinematic};
use crate::shape::Triangle;
use crate::math::Vector;

bitflags! {
    #[derive(Default)]
    pub struct HeightFieldCellStatus: u8 {
        const ZIGZAG_SUBDIVISION = 0b00000001;
        const LEFT_TRIANGLE_REMOVED = 0b00000010;
        const RIGHT_TRIANGLE_REMOVED = 0b00000100;
        const CELL_REMOVED = Self::LEFT_TRIANGLE_REMOVED.bits | Self::RIGHT_TRIANGLE_REMOVED.bits;
    }
}

#[derive(Clone, Debug)]
pub struct HeightField<N: Real> {
    heights: DMatrix<N>,
    scale: Vector<N>,
    aabb: AABB<N>,
    num_triangles: usize,
    status: DMatrix<HeightFieldCellStatus>,
}

impl<N: Real> HeightField<N> {
    pub fn new(heights: DMatrix<N>, scale: Vector<N>) -> Self {
        assert!(heights.nrows() > 1 && heights.ncols() > 1, "A heightfield heights must have at least 2 rows and columns.");
        let max = heights.max();
        let min = heights.min();
        let hscale = scale * na::convert::<_, N>(0.5);
        let aabb = AABB::new(
            Point3::new(-hscale.x, min * scale.y, -hscale.z),
            Point3::new(hscale.x, max * scale.y, hscale.z)
        );
        let num_triangles = (heights.nrows() - 1) * (heights.ncols() - 1) * 2;
        let status = DMatrix::repeat(heights.nrows() - 1, heights.ncols() - 1, HeightFieldCellStatus::default());

        HeightField {
            heights, scale, aabb, num_triangles, status
        }
    }

    pub fn nrows(&self) -> usize {
        self.heights.nrows() - 1
    }

    pub fn ncols(&self) -> usize {
        self.heights.ncols() - 1
    }

    pub fn triangle_at(&self, i: usize, j: usize, left: bool) -> Option<Triangle<N>> {
        let status = self.status[(i, j)];

        if (left && status.contains(HeightFieldCellStatus::LEFT_TRIANGLE_REMOVED)) ||
            (!left && status.contains(HeightFieldCellStatus::RIGHT_TRIANGLE_REMOVED)) {
            return None;
        }

        let cell_width = self.unit_cell_width();
        let cell_height = self.unit_cell_height();

        let _0_5: N = na::convert(0.5);
        let z0 = -_0_5 + cell_height * na::convert(i as f64);
        let z1 = z0 + cell_height;

        let x0 = -_0_5 + cell_width * na::convert(j as f64);
        let x1 = x0 + cell_width;

        let y00 = self.heights[(i + 0, j + 0)];
        let y10 = self.heights[(i + 1, j + 0)];
        let y01 = self.heights[(i + 0, j + 1)];
        let y11 = self.heights[(i + 1, j + 1)];

        let mut p00 = Point3::new(x0, y00, z0);
        let mut p10 = Point3::new(x0, y10, z1);
        let mut p01 = Point3::new(x1, y01, z0);
        let mut p11 = Point3::new(x1, y11, z1);

        // Apply scales:
        p00.coords.component_mul_mut(&self.scale);
        p10.coords.component_mul_mut(&self.scale);
        p01.coords.component_mul_mut(&self.scale);
        p11.coords.component_mul_mut(&self.scale);

        if left {
            if status.contains(HeightFieldCellStatus::ZIGZAG_SUBDIVISION) {
                Some(Triangle::new(p00, p10, p11))
            } else {
                Some(Triangle::new(p00, p10, p01))
            }
        } else {
            if status.contains(HeightFieldCellStatus::ZIGZAG_SUBDIVISION) {
                Some(Triangle::new(p00, p11, p01))
            } else {
                Some(Triangle::new(p10, p11, p01))
            }
        }
    }

    pub fn cell_status(&self, i: usize, j: usize) -> HeightFieldCellStatus {
        self.status[(i, j)]
    }

    pub fn set_cell_status(&mut self, i: usize, j: usize, status: HeightFieldCellStatus) {
        self.status[(i, j)] = status
    }

    pub fn cells_statuses(&self) -> &DMatrix<HeightFieldCellStatus> {
        &self.status
    }

    pub fn cells_statuses_mut(&mut self) -> &mut DMatrix<HeightFieldCellStatus> {
        &mut self.status
    }

    pub fn heights(&self) -> &DMatrix<N> {
        &self.heights
    }

    pub fn scale(&self) -> &Vector<N> {
        &self.scale
    }

    pub fn cell_width(&self) -> N {
        self.unit_cell_width() * self.scale.x
    }

    pub fn cell_height(&self) -> N {
        self.unit_cell_height() * self.scale.z
    }

    pub fn unit_cell_width(&self) -> N {
        N::one() / na::convert(self.heights.ncols() as f64 - 1.0)
    }

    pub fn unit_cell_height(&self) -> N {
        N::one() / na::convert(self.heights.nrows() as f64 - 1.0)
    }

    pub fn aabb(&self) -> &AABB<N> {
        &self.aabb
    }

    pub fn map_elements_in_local_aabb(&self, aabb: &AABB<N>, f: &mut impl FnMut(usize, &Triangle<N>, &ContactPreprocessor<N>)) {
        let _0_5: N = na::convert(0.5);
        let ref_mins = aabb.mins().coords.component_div(&self.scale);
        let ref_maxs = aabb.maxs().coords.component_div(&self.scale);
        let cell_width  = self.unit_cell_width();
        let cell_height = self.unit_cell_height();

        if ref_maxs.x <= -_0_5 || ref_maxs.z <= -_0_5 || ref_mins.x >= _0_5 || ref_mins.z >= _0_5 {
            // Outside of the heightfield bounds.
            return;
        }

        let min_x = unsafe { na::convert_unchecked::<N, f64>((na::clamp(ref_mins.x + _0_5, N::zero(), N::one()) / cell_width).floor()) } as usize;
        let min_z = unsafe { na::convert_unchecked::<N, f64>((na::clamp(ref_mins.z + _0_5, N::zero(), N::one()) / cell_height).floor()) } as usize;

        let max_x = unsafe { na::convert_unchecked::<N, f64>((na::clamp(ref_maxs.x + _0_5, N::zero(), N::one()) / cell_width).ceil()) } as usize;
        let max_z = unsafe { na::convert_unchecked::<N, f64>((na::clamp(ref_maxs.z + _0_5, N::zero(), N::one()) / cell_height).ceil()) } as usize;

        // FIXME: find a way to avoid recomputing the same vertices
        // multiple times.
        for j in min_x..max_x {
            for i in min_z..max_z {
                let status = self.status[(i, j)];

                if status.contains(HeightFieldCellStatus::CELL_REMOVED) {
                    continue;
                }

                let z0 = -_0_5 + cell_height * na::convert(i as f64);
                let z1 = z0 + cell_height;

                let x0 = -_0_5 + cell_width * na::convert(j as f64);
                let x1 = x0 + cell_width;

                let y00 = self.heights[(i + 0, j + 0)];
                let y10 = self.heights[(i + 1, j + 0)];
                let y01 = self.heights[(i + 0, j + 1)];
                let y11 = self.heights[(i + 1, j + 1)];

                if (y00 > ref_maxs.y && y10 > ref_maxs.y && y01 > ref_maxs.y && y11 > ref_maxs.y) ||
                    (y00 < ref_mins.y && y10 < ref_mins.y && y01 < ref_mins.y && y11 < ref_mins.y) {
                    continue;
                }

                let mut p00 = Point3::new(x0, y00, z0);
                let mut p10 = Point3::new(x0, y10, z1);
                let mut p01 = Point3::new(x1, y01, z0);
                let mut p11 = Point3::new(x1, y11, z1);

                // Apply scales:
                p00.coords.component_mul_mut(&self.scale);
                p10.coords.component_mul_mut(&self.scale);
                p01.coords.component_mul_mut(&self.scale);
                p11.coords.component_mul_mut(&self.scale);

                // Build the two triangles, contact processors and call f.
                let triangle_id1 = j * (self.heights.nrows() - 1) + i;

                if !status.contains(HeightFieldCellStatus::LEFT_TRIANGLE_REMOVED) {
                    let tri1 = if status.contains(HeightFieldCellStatus::ZIGZAG_SUBDIVISION) {
                        Triangle::new(p00, p10, p11)
                    } else {
                        Triangle::new(p00, p10, p01)
                    };

                    let proc1 = HeightFieldTriangleContactPreprocessor::new(self, triangle_id1);
                    f(triangle_id1, &tri1, &proc1);
                }

                if !status.contains(HeightFieldCellStatus::RIGHT_TRIANGLE_REMOVED) {
                    let tri2 = if status.contains(HeightFieldCellStatus::ZIGZAG_SUBDIVISION) {
                        Triangle::new(p00, p11, p01)
                    } else {
                        Triangle::new(p10, p11, p01)
                    };
                    let triangle_id2 = triangle_id1 + self.num_triangles / 2;
                    let proc2 = HeightFieldTriangleContactPreprocessor::new(self, triangle_id2);
                    f(triangle_id2, &tri2, &proc2);
                }
            }
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