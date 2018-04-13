use std::f64;
#[cfg(feature = "dim3")]
use smallvec::SmallVec;

use na::{self, Real, Unit};
use math::{Vector, Isometry};

#[cfg(feature = "dim2")]
#[derive(Clone, Debug)]
pub enum PolyhedralCone<N: Real> {
    Full,
    Empty,
    HalfLine(Unit<Vector<N>>),
    HalfSpace(Unit<Vector<N>>),
    OrthogonalSubspace(Unit<Vector<N>>),
    Span([Unit<Vector<N>>; 2])
}

#[cfg(feature = "dim3")]
#[derive(Clone, Debug)]
pub enum PolyhedralCone<N: Real> {
    Full,
    Empty,
    HalfLine(Unit<Vector<N>>),
    HalfSpace(Unit<Vector<N>>),
    OrthogonalSubspace(Unit<Vector<N>>),
    Span(SmallVec<[Unit<Vector<N>>; 4]>)
}

impl<N: Real> PolyhedralCone<N> {
    pub fn unwrap_half_line(&self) -> Unit<Vector<N>> {
        if let PolyhedralCone::HalfLine(dir) = *self {
            dir
        } else {
            panic!("This polyhedral cone is not a half-line.")
        }
    }
    
    pub fn transform_by(&mut self, m: &Isometry<N>) {
        match *self {
                PolyhedralCone::HalfLine(ref mut dir) => *dir = m * *dir,
                PolyhedralCone::HalfSpace(ref mut normal) => *normal = m * *normal,
                PolyhedralCone::OrthogonalSubspace(ref mut normal) => *normal = m * *normal,
                PolyhedralCone::Span(ref mut generators) => {
                    for gen in generators {
                        *gen = m * *gen;
                    }
                }
                PolyhedralCone::Full => {}
                PolyhedralCone::Empty => {}
        }
    }

    pub fn contains(&self, v: &Vector<N>) -> bool {
        if let Some(dir) = Unit::try_new(*v, N::default_epsilon()) {
            self.contains_dir(&dir)
        } else {
            true
        }
    }

    pub fn polar_contains_dir(&self, dir: &Unit<Vector<N>>) -> bool {
        let eps: N = na::convert(f64::consts::PI / 180.0 * 0.01);
        let c_eps = eps.cos();
        
        match *self {
            PolyhedralCone::Full => false,
            PolyhedralCone::Empty => true,
            PolyhedralCone::HalfLine(ref generator) => na::dot(generator.as_ref(), dir.as_ref()) <= na::zero(),
            PolyhedralCone::HalfSpace(ref normal) => na::dot(normal.as_ref(), dir.as_ref()) >= na::zero(),
            PolyhedralCone::OrthogonalSubspace(ref normal) => na::dot(normal.as_ref(), dir.as_ref()).abs() >= N::one() - c_eps,
            PolyhedralCone::Span(ref generators) => {
                for g in generators {
                    if na::dot(g.as_ref(), dir.as_ref()) > na::zero() {
                        return false;
                    }
                }
                true
            }
        }
    }

    pub fn contains_dir(&self, dir: &Unit<Vector<N>>) -> bool {
        let eps: N = na::convert(f64::consts::PI / 180.0 * 0.01);
        let c_eps = eps.cos();
        
        match *self {
            PolyhedralCone::Full => true,
            PolyhedralCone::Empty => false,
            PolyhedralCone::HalfLine(ref generator) => na::dot(generator.as_ref(), dir.as_ref()) >= N::one() - c_eps,
            PolyhedralCone::HalfSpace(ref normal) => na::dot(normal.as_ref(), dir.as_ref()) <= na::zero(),
            PolyhedralCone::OrthogonalSubspace(ref normal) => na::dot(normal.as_ref(), dir.as_ref()).abs() <= c_eps,
            PolyhedralCone::Span(ref generators) => {
                #[cfg(feature = "dim2")]                
                {
                    // NOTE: the following assumes the polycone
                    // generator are ordered in CCW order.
                    let perp1 = dir.as_ref().perp(&*generators[0]);
                    let perp2 = dir.as_ref().perp(&*generators[1]);

                    perp1 <= N::zero() && perp2 >= N::zero()
                }
                
                #[cfg(feature = "dim3")]
                {
                    // NOTE: the following does not makes any assumptions on the
                    // polycone orientation.
                    if generators.len() == 1 {
                        // The polycone is degenerate and actually has only one generactor.
                        let eps: N = na::convert(f64::consts::PI / 180.0 * 0.1);
                        let c_eps = eps.cos();
                        let dot = na::dot(&*generators[0], dir.as_ref());
                        dot >= c_eps
                    } else if generators.len() == 2 {
                        let eps = na::convert(f64::consts::PI / 180.0 * 0.1);
                        let normal = generators[1].cross(&*generators[0]);

                        if let Some(normal) = Unit::try_new(normal, na::zero()) {
                            if na::dot(&*normal, dir.as_ref()).abs() > eps {
                                return false;
                            }

                            let middle = (*generators[0] + *generators[1]) * na::convert::<_, N>(0.5);
                            if na::dot(&middle, dir.as_ref()) < na::zero() {
                                return false;
                            }

                            let cross1 = generators[0].cross(dir.as_ref());
                            let cross2 = generators[1].cross(dir.as_ref());

                            na::dot(&cross1, &*normal) * na::dot(&cross2, &*normal) <= na::zero()
                        } else {
                            // FIXME: duplicate code with the case where we only have one generator.
                            // The polycone is degenerate and actually has only one generactor.
                            let c_eps = eps.cos();
                            let dot = na::dot(&*generators[0], dir.as_ref());
                            dot >= c_eps
                        }
                    } else {
                        let mut sign = N::zero();
                        let mut center = Vector::zeros();

                        for i1 in 0..generators.len() {
                            let i2 = (i1 + 1) % generators.len();
                            let cross = generators[i1].cross(generators[i2].as_ref());
                            let dot = na::dot(dir.as_ref(), &cross);
                            center += generators[i1].unwrap();

                            if sign.is_zero() {
                                sign = dot
                            } else if sign * dot < na::zero() {
                                return false;
                            }
                        }

                        // FIXME: is this a sufficient condition to determine if the
                        // dir is not of the opposite cone?
                        na::dot(&center, dir.as_ref()) >= na::zero()
                    }
                }
            }
        }
    }
}
