use na::{Vec2, Mat2, Translate, Transform, Inv, Norm};
use na;
use geom::BezierSurface;
use geom::Ball;
use narrow::{CollisionDetector, Contact};
use narrow::surface_selector::SurfaceSelector;
use math::{Scalar, Point, Vect};


/// Collision detector between a ball and a bezier surface.
pub struct BallBezierSurface<N, P, V, M, S, D> {
    selector:   S,
    prediction: N,
    contacts:   Vec<Contact<N, P, V>>,
    points:     Vec<P>,
}

impl<N: Clone, P: Clone, V: Clone, M, S: Clone, D: Send + Sync> Clone for BallBezierSurface<N, P, V, M, S, D> {
    fn clone(&self) -> BallBezierSurface<N, P, V, M, S, D> {
        BallBezierSurface {
            selector:   self.selector.clone(),
            prediction: self.prediction.clone(),
            contacts:   self.contacts.clone(),
            points:     self.points.clone()
        }
    }
}

impl<N, P, V, M, S: SurfaceSelector<N, P, D>, D> BallBezierSurface<N, P, V, M, S, D> {
    /// Creates a new collision detector with the given prediction margin.
    pub fn new(selector: S, prediction: N) -> BallBezierSurface<N, P, V, M, S, D> {
        BallBezierSurface {
            selector:   selector,
            prediction: prediction,
            contacts:   Vec::new(),
            points:     Vec::new()
        }
    }
}

impl<N, P, V, M, S, D> CollisionDetector<N, P, V, M, Ball<N>, BezierSurface<P>> for BallBezierSurface<N, P, V, M, S, D>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Transform<P>,
          S: SurfaceSelector<N, P, D>,
          D: Send + Sync {
    fn update(&mut self, ma: &M, a: &Ball<N>, mb: &M, b: &BezierSurface<P>) {
        self.points.clear();
        self.contacts.clear();
        self.selector.set_max_lmd(a.radius() + self.prediction);

        let max_depth = 15;
        let niter     = 5;
        let pt        = na::inv_transform(mb, &ma.translate(&na::orig()));

        closest_points(&pt, b, niter, &mut self.selector, max_depth, &mut self.points);

        for pt in self.points.iter() {
            let pt = na::transform(mb, pt);
            let mut normal = pt - ma.translate(&na::orig());
            let gap        = normal.normalize();

            let c = Contact::new(ma.translate(&na::orig()) + normal * a.radius(), pt.clone(), normal, a.radius() - gap);

            self.contacts.push(c);
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.contacts.len()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        out_colls.push_all(self.contacts.as_slice());
    }
}

/// Collision detector between a bezier surface and a ball.
pub struct BezierSurfaceBall<N, P, V, M, S, D> {
    detector: BallBezierSurface<N, P, V, M, S, D>
}

impl<N: Clone, P: Clone, V: Clone, M, S: Clone, D: Send + Sync> Clone for BezierSurfaceBall<N, P, V, M, S, D> {
    fn clone(&self) -> BezierSurfaceBall<N, P, V, M, S, D> {
        BezierSurfaceBall {
            detector: self.detector.clone()
        }
    }
}

impl<N, P, V, M, S: SurfaceSelector<N, P, D>, D> BezierSurfaceBall<N, P, V, M, S, D> {
    /// Creates a new collision detector with the given prediction margin.
    pub fn new(selector: S, prediction: N) -> BezierSurfaceBall<N, P, V, M, S, D> {
        BezierSurfaceBall {
            detector: BallBezierSurface::new(selector, prediction)
        }
    }
}

impl<N, P, V, M, S, D> CollisionDetector<N, P, V, M, BezierSurface<P>, Ball<N>> for BezierSurfaceBall<N, P, V, M, S, D>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Transform<P>,
          S: SurfaceSelector<N, P, D>,
          D: Send + Sync {
    fn update(&mut self, ma: &M, a: &BezierSurface<P>, mb: &M, b: &Ball<N>) {
        self.detector.update(mb, b, ma, a);

        for c in self.detector.contacts.iter_mut() {
            c.flip();
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.detector.num_colls()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        self.detector.colls(out_colls)
    }
}

/// Computes the points of a bézier surface `b` closest to `pt`.
///
/// # Arguments:
/// * `pt`        - The point to project.
/// * `b`         - The bézier surface.
/// * `niter`     - The number of iteration for the numerical solver.
/// * `selector`  - The surface selection subalgorithm.
/// * `max_depth` - The maximum depth of the subdivision tree.
/// * `out`       - The buffer that will contain the closest points.
pub fn closest_points<N, P, V, S, D>(pt:        &P,
                                     b:         &BezierSurface<P>,
                                     niter:     uint,
                                     selector:  &mut S,
                                     max_depth: uint,
                                     out:       &mut Vec<P>)
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          S: SurfaceSelector<N, P, D> {
    do_closest_points(pt, b, niter, selector, max_depth, out, false)
}


fn do_closest_points<N, P, V, S, D>(pt:        &P,
                                    b:         &BezierSurface<P>,
                                    niter:     uint,
                                    selector:  &mut S,
                                    max_depth: uint,
                                    out:       &mut Vec<P>,
                                    odd:       bool)
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          S: SurfaceSelector<N, P, D> {
    let test_data = selector.create_test_data(b);

    if max_depth == 0 || selector.is_flat(b, &test_data) {
        // stop the recursion and get the beloved answer!
        match closest_point(pt, b, niter) {
            Some(pt) => out.push(pt),
            None     => { }
        }
    }
    else if selector.may_contain_a_closest_point(pt, b, &test_data) {
        // subdivide
        let mut left  = BezierSurface::new_with_degrees(b.degree_u(), b.degree_v());
        let mut right = BezierSurface::new_with_degrees(b.degree_u(), b.degree_v());

        if odd {
            b.subdivide_u(na::cast(0.5f64), &mut left, &mut right);
        }
        else {
            b.subdivide_v(na::cast(0.5f64), &mut left, &mut right);
        }

        do_closest_points(pt, &left,  niter, selector, max_depth - 1, out, !odd);
        do_closest_points(pt, &right, niter, selector, max_depth - 1, out, !odd);
    }

    // otherwise there is no solution.
}

// FIXME: the Newton method should be implemented on nalgebra.
fn closest_point<N, P, V>(pt: &P, b: &BezierSurface<P>, niter: uint) -> Option<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    /*
     * derivatives
     */
    let mut diff_u = BezierSurface::new_with_degrees(b.degree_u() - 1, b.degree_v());
    let mut diff_v = BezierSurface::new_with_degrees(b.degree_u()    , b.degree_v() - 1);

    /*
     * second diffatives
     */
    let mut diff_u_u = BezierSurface::new_with_degrees(b.degree_u() - 2, b.degree_v());
    let mut diff_v_v = BezierSurface::new_with_degrees(b.degree_u()    , b.degree_v() - 2);

    let mut diff_u_v = BezierSurface::new_with_degrees(b.degree_u() - 1, b.degree_v() - 1);
    let mut diff_v_u = BezierSurface::new_with_degrees(b.degree_u() - 1, b.degree_v() - 1);

    /*
     * Compute all the derivatives (needed by the jacobian).
     */
    b.diff_u(&mut diff_u);
    b.diff_v(&mut diff_v);

    diff_u.diff_u(&mut diff_u_u);
    diff_u.diff_v(&mut diff_u_v);
    diff_v.diff_u(&mut diff_v_u);
    diff_v.diff_v(&mut diff_v_v);

    /*
     * Newton method.
     */
    let mut uv: Vec2<N> = Vec2::new(na::cast(0.5f64), na::cast(0.5f64));

    let mut cache = BezierSurface::new_evaluation_cache();

    for _ in range(0, niter) {
        // compute the jacobian
        let dpt = b.at(uv.x, uv.y, &mut cache) - *pt;
        let du  = diff_u.at(uv.x, uv.y, &mut cache);
        let dv  = diff_v.at(uv.x, uv.y, &mut cache);
        let duu = diff_u_u.at(uv.x, uv.y, &mut cache);
        let duv = diff_u_v.at(uv.x, uv.y, &mut cache);
        let dvv = diff_v_v.at(uv.x, uv.y, &mut cache);
        let dvu = diff_v_u.at(uv.x, uv.y, &mut cache);

        let df_uu = na::dot(duu.as_vec(), &dpt) + na::dot(du.as_vec(), du.as_vec());
        let df_uv = na::dot(duv.as_vec(), &dpt) + na::dot(du.as_vec(), dv.as_vec());
        let df_vv = na::dot(dvv.as_vec(), &dpt) + na::dot(dv.as_vec(), dv.as_vec());
        let df_vu = na::dot(dvu.as_vec(), &dpt) + na::dot(dv.as_vec(), du.as_vec());

        let f_u = na::dot(&dpt, du.as_vec());
        let f_v = na::dot(&dpt, dv.as_vec());

        let f         = Vec2::new(f_u, f_v);
        let mut inv_j = Mat2::new(df_uu, df_uv,
                                  df_vu, df_vv);

        if !inv_j.inv() {
            return None;
        }

        uv = uv - inv_j * f;

        // we allow smalls steps outside of the domain boundaries.
        if uv.x < na::cast(-1.0f64) || uv.y < na::cast(-1.0f64) ||
           uv.x > na::cast(2.0f64)  || uv.y > na::cast(2.0f64) {
                return None
        }
    }

    if uv.x < na::zero() || uv.y < na::zero() ||
       uv.x > na::one()  || uv.y > na::one() {
        return None
    }

    Some(b.at(uv.x, uv.y, &mut cache))
}
