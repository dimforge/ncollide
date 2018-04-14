use math::Point;
use na::{self, Id};
use query::{PointQuery, PointQueryWithLocation};
use query::algorithms::simplex::Simplex;
use shape::{Segment, SegmentPointLocation, Triangle, TrianglePointLocation};

/// A simplex of dimension up to 2 using Voronoï regions for computing point projections.
pub struct VoronoiSimplex2<N: Real> {
    vertices: [P; 3],
    dim: usize,
}

impl<N: Real> VoronoiSimplex2<P> {
    /// Crates a new empty simplex.
    pub fn new() -> VoronoiSimplex2<P> {
        VoronoiSimplex2 {
            vertices: [Point::origin(); 3],
            dim: 0,
        }
    }
}

/// Trait of a simplex usable by the GJK algorithm.
impl<N: Real> Simplex<P> for VoronoiSimplex2<P> {
    fn reset(&mut self, pt: Point<N>) {
        self.dim = 0;
        self.vertices[0] = pt;
    }

    fn add_point(&mut self, pt: Point<N>) -> bool {
        for i in 0..self.dim + 1 {
            if self.vertices[i].coords == pt.coords {
                return false;
            }
        }

        self.dim += 1;
        self.vertices[self.dim] = pt;
        return true;
    }

    fn point(&self, i: usize) -> Point<N> {
        assert!(i <= self.dim, "Index out of bounds.");
        self.vertices[i]
    }

    fn project_origin_and_reduce(&mut self) -> Point<N> {
        if self.dim == 0 {
            self.vertices[0]
        } else if self.dim == 1 {
            // FIXME: NLL
            let (proj, location) = {
                let seg = Segment::from_array3(&self.vertices);
                seg.project_point_with_location(&Id::new(), &Point::origin(), true)
            };

            match location {
                SegmentPointLocation::OnVertex(0) => {
                    self.dim = 0;
                }
                SegmentPointLocation::OnVertex(1) => {
                    self.vertices[0] = self.vertices[1];
                    self.dim = 0;
                }
                _ => {}
            }

            proj.point
        } else {
            assert!(self.dim == 2);
            // FIXME: NLL
            let (proj, location) = {
                let tri = Triangle::from_array(&self.vertices);
                tri.project_point_with_location(&Id::new(), &Point::origin(), true)
            };

            match location {
                TrianglePointLocation::OnVertex(i) => {
                    self.vertices[0] = self.vertices[i];
                    self.dim = 0;
                }
                TrianglePointLocation::OnEdge(0, _) => {
                    self.dim = 1;
                }
                TrianglePointLocation::OnEdge(1, _) => {
                    self.vertices[0] = self.vertices[2];
                    self.dim = 1;
                }
                TrianglePointLocation::OnEdge(2, _) => {
                    self.vertices[1] = self.vertices[2];
                    self.dim = 1;
                }
                _ => {}
            }

            proj.point
        }
    }

    fn project_origin(&mut self) -> Point<N> {
        if self.dim == 0 {
            self.vertices[0]
        } else if self.dim == 1 {
            let seg = Segment::from_array3(&self.vertices);
            seg.project_point(&Id::new(), &Point::origin(), true).point
        } else {
            assert!(self.dim == 2);
            let tri = Triangle::from_array(&self.vertices);
            tri.project_point(&Id::new(), &Point::origin(), true).point
        }
    }

    fn contains_point(&self, pt: &P) -> bool {
        for i in 0..self.dim + 1 {
            if self.vertices[i] == *pt {
                return true;
            }
        }

        false
    }

    fn dimension(&self) -> usize {
        self.dim
    }

    fn max_sq_len(&self) -> N {
        let mut max_sq_len = na::zero();

        for i in 0..self.dim + 1 {
            let norm = na::norm_squared(&self.vertices[i].coords);

            if norm > max_sq_len {
                max_sq_len = norm
            }
        }

        max_sq_len
    }

    fn modify_pnts(&mut self, f: &Fn(&mut P)) {
        for i in 0..self.dim + 1 {
            f(&mut self.vertices[i])
        }
    }
}
