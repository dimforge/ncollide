use na::{self, Real};
use math::{Point, Isometry};
use query::{PointQuery, PointQueryWithLocation};
use query::algorithms::{CSOPoint, simplex::Simplex};
use shape::{Segment, SegmentPointLocation, Triangle, TrianglePointLocation};

/// A simplex of dimension up to 2 using Voronoï regions for computing point projections.
pub struct VoronoiSimplex<N: Real> {
    prev_vertices: [usize; 3],
    prev_dim: usize,
    prev_proj: [N; 2],

    vertices: [CSOPoint<N>; 3],
    proj: [N; 2],
    dim: usize,
}

impl<N: Real> VoronoiSimplex<N> {
    /// Crates a new empty simplex.
    pub fn new() -> VoronoiSimplex<N> {
        VoronoiSimplex {
            prev_vertices: [0, 1, 2],
            prev_proj: [N::zero(); 2],
            prev_dim: 0,
            vertices: [CSOPoint::origin(); 3],
            proj: [N::zero(); 2],
            dim: 0,
        }
    }

    fn swap(&mut self, i1: usize, i2: usize) {
        self.vertices.swap(i1, i2);
        self.prev_vertices.swap(i1, i2);
    }
}

/// Trait of a simplex usable by the GJK algorithm.
impl<N: Real> Simplex<N> for VoronoiSimplex<N> {
    fn reset(&mut self, pt: CSOPoint<N>) {
        self.prev_dim = 0;
        self.dim = 0;
        self.vertices[0] = pt;
    }

    fn add_point(&mut self, pt: CSOPoint<N>) -> bool {
        self.prev_dim = self.dim;
        self.prev_proj = self.proj;
        self.prev_vertices = [0, 1, 2];

        for i in 0..self.dim + 1 {
            if self.vertices[i].point == pt.point {
                return false;
            }
        }

        self.dim += 1;
        self.vertices[self.dim] = pt;
        return true;
    }

    fn proj_coord(&self, i: usize) -> N {
        assert!(i <= self.dim, "Index out of bounds.");
        self.proj[i]
    }

    fn point(&self, i: usize) -> &CSOPoint<N> {
        assert!(i <= self.dim, "Index out of bounds.");
        &self.vertices[i]
    }

    fn prev_proj_coord(&self, i: usize) -> N {
        assert!(i <= self.dim, "Index out of bounds.");
        self.prev_proj[i]
    }
    
    fn prev_point(&self, i: usize) -> &CSOPoint<N> {
        assert!(i <= self.prev_dim, "Index out of bounds.");
        &self.vertices[self.prev_vertices[i]]
    }

    fn project_origin_and_reduce(&mut self) -> Point<N> {
        if self.dim == 0 {
            self.proj[0] = N::one();
            self.vertices[0].point
        } else if self.dim == 1 {
            // FIXME: NLL
            let (proj, location) = {
                let seg = Segment::new(self.vertices[0].point, self.vertices[1].point);
                seg.project_point_with_location(&Isometry::identity(), &Point::origin(), true)
            };

            match location {
                SegmentPointLocation::OnVertex(0) => {
                    self.proj[0] = N::one();
                    self.dim = 0;
                }
                SegmentPointLocation::OnVertex(1) => {
                    self.proj[0] = N::one();
                    self.swap(0, 1);
                    self.dim = 0;
                }
                SegmentPointLocation::OnEdge(coords) => {
                    self.proj = coords;
                },
                _ => unreachable!()
            }

            proj.point
        } else {
            assert!(self.dim == 2);
            // FIXME: NLL
            let (proj, location) = {
                let tri = Triangle::new(self.vertices[0].point, self.vertices[1].point, self.vertices[2].point);
                tri.project_point_with_location(&Isometry::identity(), &Point::origin(), true)
            };

            match location {
                TrianglePointLocation::OnVertex(i) => {
                    self.swap(0, i);
                    self.proj[0] = N::one();
                    self.dim = 0;
                }
                TrianglePointLocation::OnEdge(0, coords) => {
                    self.proj = coords;
                    self.dim = 1;
                }
                TrianglePointLocation::OnEdge(1, coords) => {
                    self.swap(0, 2);
                    self.proj[0] = coords[1];
                    self.proj[1] = coords[0];
                    self.dim = 1;
                }
                TrianglePointLocation::OnEdge(2, coords) => {
                    self.swap(1, 2);
                    self.proj = coords;
                    self.dim = 1;
                }
                _ => {}
            }

            proj.point
        }
    }

    fn project_origin(&mut self) -> Point<N> {
        if self.dim == 0 {
            self.vertices[0].point
        } else if self.dim == 1 {
            let seg = Segment::new(self.vertices[0].point, self.vertices[1].point);
            seg.project_point(&Isometry::identity(), &Point::origin(), true).point
        } else {
            assert!(self.dim == 2);
            let tri = Triangle::new(self.vertices[0].point, self.vertices[1].point, self.vertices[2].point);
            tri.project_point(&Isometry::identity(), &Point::origin(), true).point
        }
    }

    fn contains_point(&self, pt: &Point<N>) -> bool {
        for i in 0..self.dim + 1 {
            if self.vertices[i].point == *pt {
                return true;
            }
        }

        false
    }

    fn dimension(&self) -> usize {
        self.dim
    }

    fn prev_dimension(&self) -> usize {
        self.prev_dim
    }

    fn max_sq_len(&self) -> N {
        let mut max_sq_len = na::zero();

        for i in 0..self.dim + 1 {
            let norm = na::norm_squared(&self.vertices[i].point.coords);

            if norm > max_sq_len {
                max_sq_len = norm
            }
        }

        max_sq_len
    }

    fn modify_pnts(&mut self, f: &Fn(&mut CSOPoint<N>)) {
        for i in 0..self.dim + 1 {
            f(&mut self.vertices[i])
        }
    }
}
