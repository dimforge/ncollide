use math::Point;
use alga::general::Real;
use na::{self, Id};
use query::{PointQuery, PointQueryWithLocation};
use query::algorithms::simplex::Simplex;
use query::algorithms::gjk;
use shape::{Segment, SegmentPointLocation, Tetrahedron, TetrahedronPointLocation, Triangle,
            TrianglePointLocation};
use utils;

/// A simplex of dimension up to 3 that uses Voronoï regions for computing point projections.
pub struct VoronoiSimplex3<P: Point> {
    vertices: [P; 4],
    dim: usize,
}

impl<P: Point> VoronoiSimplex3<P> {
    /// Creates a new empty simplex.
    pub fn new() -> VoronoiSimplex3<P> {
        VoronoiSimplex3 {
            vertices: [P::origin(); 4],
            dim: 0,
        }
    }
}

/// Trait of a simplex usable by the GJK algorithm.
impl<P: Point> Simplex<P> for VoronoiSimplex3<P> {
    fn reset(&mut self, pt: P) {
        self.dim = 0;
        self.vertices[0] = pt;
    }

    fn add_point(&mut self, pt: P) -> bool {
        match self.dim {
            0 => {
                if na::norm_squared(&(self.vertices[0] - pt)) < gjk::eps_tol() {
                    return false;
                }
            }
            1 => {
                let ab = self.vertices[1] - self.vertices[0];
                let ac = pt - self.vertices[0];

                if na::norm_squared(&utils::cross3(&ab, &ac)) < gjk::eps_tol() {
                    return false;
                }
            }
            2 => {
                let ab = self.vertices[1] - self.vertices[0];
                let ac = self.vertices[2] - self.vertices[0];
                let ap = pt - self.vertices[0];
                let n = na::normalize(&utils::cross3(&ab, &ac));

                if na::dot(&n, &ap).abs() < gjk::eps_tol() {
                    return false;
                }
            }
            _ => unreachable!(),
        }

        self.dim += 1;
        self.vertices[self.dim] = pt;
        return true;
    }

    fn point(&self, i: usize) -> P {
        assert!(i <= self.dim, "Index out of bounds.");
        self.vertices[i]
    }

    fn project_origin_and_reduce(&mut self) -> P {
        if self.dim == 0 {
            self.vertices[0]
        } else if self.dim == 1 {
            // FIXME: NLL
            let (proj, location) = {
                let seg = Segment::from_array4(&self.vertices);
                seg.project_point_with_location(&Id::new(), &P::origin(), true)
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
        } else if self.dim == 2 {
            // FIXME: NLL
            let (proj, location) = {
                let tri = Triangle::from_array4(&self.vertices);
                tri.project_point_with_location(&Id::new(), &P::origin(), true)
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
        } else {
            assert!(self.dim == 3);
            // FIXME: NLL
            let (proj, location) = {
                let tetr = Tetrahedron::from_array(&self.vertices);
                tetr.project_point_with_location(&Id::new(), &P::origin(), true)
            };

            match location {
                TetrahedronPointLocation::OnVertex(i) => {
                    self.vertices[0] = self.vertices[i];
                    self.dim = 0;
                }
                TetrahedronPointLocation::OnEdge(i, _) => {
                    match i {
                        0 => {
                            // ab
                        }
                        1 => {
                            // ac
                            self.vertices[1] = self.vertices[2];
                        }
                        2 => {
                            // ad
                            self.vertices[1] = self.vertices[3];
                        }
                        3 => {
                            // bc
                            self.vertices[0] = self.vertices[2];
                        }
                        4 => {
                            // bd
                            self.vertices[0] = self.vertices[3];
                        }
                        5 => {
                            // cd
                            self.vertices[0] = self.vertices[2];
                            self.vertices[1] = self.vertices[3];
                        }
                        _ => unreachable!(),
                    }
                    self.dim = 1;
                }
                TetrahedronPointLocation::OnFace(i, _) => {
                    match i {
                        0 => {
                            // abc
                        }
                        1 => {
                            // abd
                            self.vertices[2] = self.vertices[3];
                        }
                        2 => {
                            // acd
                            self.vertices[1] = self.vertices[3];
                        }
                        3 => {
                            // bcd
                            self.vertices[0] = self.vertices[3];
                        }
                        _ => unreachable!(),
                    }
                    self.dim = 2;
                }
                _ => {}
            }

            proj.point
        }
    }

    fn project_origin(&mut self) -> P {
        if self.dim == 0 {
            self.vertices[0]
        } else if self.dim == 1 {
            let seg = Segment::from_array4(&self.vertices);
            seg.project_point(&Id::new(), &P::origin(), true).point
        } else if self.dim == 2 {
            let tri = Triangle::from_array4(&self.vertices);
            tri.project_point(&Id::new(), &P::origin(), true).point
        } else {
            let tetr = Tetrahedron::from_array(&self.vertices);
            tetr.project_point(&Id::new(), &P::origin(), true).point
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

    fn max_sq_len(&self) -> P::Real {
        let mut max_sq_len = na::zero();

        for i in 0..self.dim + 1 {
            let norm = na::norm_squared(&self.vertices[i].coordinates());

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
