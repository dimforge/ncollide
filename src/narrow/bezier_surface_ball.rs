use nalgebra::na;
use nalgebra::na::{Vec2, Mat2, Translation, Inv, Norm};
use math::{Scalar, Vect, Matrix};
use geom::{Ball, BezierSurface};
use narrow::CollisionDetector;
use contact::Contact;
use narrow::surface_selector::SurfaceSelector;

/// Collision detector between a ball and a bezier surface.
#[deriving(Clone)]
pub struct BallBezierSurface<S, D> {
    // subdivision_tree: Arc<RWLock<SurfaceSubdivisionTree<C>>,
    selector:         S,
    prediction:       Scalar,
    contacts:         Vec<Contact>,
    points:           Vec<Vect>
}

impl<S: SurfaceSelector<D>, D> BallBezierSurface<S, D> {
    /// Creates a new collision detector with the given prediction margin.
    pub fn new(selector: S, prediction: Scalar) -> BallBezierSurface<S, D> {
        BallBezierSurface {
            selector:   selector,
            prediction: prediction,
            contacts:   Vec::new(),
            points:     Vec::new(),
        }
    }
}

impl<S: SurfaceSelector<D>, D> CollisionDetector<Ball, BezierSurface> for BallBezierSurface<S, D> {
    fn update(&mut self, ma: &Matrix, a: &Ball, mb: &Matrix, b: &BezierSurface) {
        self.points.clear();
        self.contacts.clear();
        self.selector.set_max_lmd(a.radius() + self.prediction);
        closest_points(&na::inv_transform(mb, &ma.translation()), b, 10, &mut self.selector, 15, &mut self.points);

        for pt in self.points.iter() {
            let pt = na::transform(mb, pt);
            let mut normal = pt - ma.translation(); 
            let gap        = normal.normalize();

            let c = Contact::new(ma.translation() + normal * a.radius(), pt.clone(), normal, a.radius() - gap);

            self.contacts.push(c);
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.contacts.len()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact>) {
        for c in self.contacts.iter() {
            out_colls.push(c.clone())
        }
    }

    #[inline]
    fn toi(_: Option<BallBezierSurface<S, D>>, _: &Matrix, _: &Vect, _: &Scalar, _: &Ball, _: &Matrix, _: &BezierSurface) -> Option<Scalar> {
        None
    }
}

/// Collision detector between a bezier surface and a ball.
#[deriving(Clone)]
pub struct BezierSurfaceBall<S, D> {
    selector:   S,
    prediction: Scalar,
    contacts:   Vec<Contact>,
    points:     Vec<Vect>
}

impl<S: SurfaceSelector<D>, D> BezierSurfaceBall<S, D> {
    /// Creates a new collision detector with the given prediction margin.
    pub fn new(selector: S, prediction: Scalar) -> BezierSurfaceBall<S, D> {
        BezierSurfaceBall {
            selector:   selector,
            prediction: prediction,
            contacts:   Vec::new(),
            points:     Vec::new(),
        }
    }
}

impl<S: SurfaceSelector<D>, D> CollisionDetector<BezierSurface, Ball> for BezierSurfaceBall<S, D> {
    fn update(&mut self, ma: &Matrix, a: &BezierSurface, mb: &Matrix, b: &Ball) {
        self.points.clear();
        self.contacts.clear();
        self.selector.set_max_lmd(b.radius() + self.prediction);
        closest_points(&na::inv_transform(ma, &mb.translation()), a, 10, &mut self.selector, 15, &mut self.points);

        for pt in self.points.iter() {
            let pt         = na::transform(ma, pt);
            let mut normal = pt - mb.translation(); 
            let gap        = normal.normalize();

            let mut c = Contact::new(mb.translation() + normal * b.radius(), pt.clone(), normal, b.radius() - gap);

            c.flip();

            self.contacts.push(c);
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.contacts.len()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact>) {
        for c in self.contacts.iter() {
            out_colls.push(c.clone())
        }
    }

    #[inline]
    fn toi(_: Option<BezierSurfaceBall<S, D>>, _: &Matrix, _: &Vect, _: &Scalar, _: &BezierSurface, _: &Matrix, _: &Ball) -> Option<Scalar> {
        None
    }
}

fn closest_points<S: SurfaceSelector<D>, D>(pt:        &Vect,
                                            b:         &BezierSurface,
                                            niter:     uint,
                                            selector:  &mut S,
                                            max_depth: uint,
                                            out:       &mut Vec<Vect>) {
    do_closest_points(pt, b, niter, selector, max_depth, out, false)
}


fn do_closest_points<S: SurfaceSelector<D>, D>(pt:        &Vect,
                                               b:         &BezierSurface,
                                               niter:     uint,
                                               selector:  &mut S,
                                               max_depth: uint,
                                               out:       &mut Vec<Vect>,
                                               odd:       bool) {
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
            b.subdivide_u(&na::cast(0.5), &mut left, &mut right);
        }
        else {
            b.subdivide_v(&na::cast(0.5), &mut left, &mut right);
        }

        do_closest_points(pt, &left,  niter, selector, max_depth - 1, out, !odd);
        do_closest_points(pt, &right, niter, selector, max_depth - 1, out, !odd);
    }

    // otherwise there is no solution.
}

// FIXME: the Newton method should be implemented on nalgebra.
fn closest_point(pt: &Vect, b: &BezierSurface, niter: uint) -> Option<Vect> {
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
    let mut uv: Vec2<Scalar> = Vec2::new(na::cast(0.5), na::cast(0.5));

    let mut cache = BezierSurface::new_evaluation_cache();

    for _ in range(0, niter) {
        // compute the jacobian
        let dpt = b.at(&uv.x, &uv.y, &mut cache) - *pt;
        let du  = diff_u.at(&uv.x, &uv.y, &mut cache);
        let dv  = diff_v.at(&uv.x, &uv.y, &mut cache);
        let duu = diff_u_u.at(&uv.x, &uv.y, &mut cache);
        let duv = diff_u_v.at(&uv.x, &uv.y, &mut cache);
        let dvv = diff_v_v.at(&uv.x, &uv.y, &mut cache);
        let dvu = diff_v_u.at(&uv.x, &uv.y, &mut cache);

        let df_uu = na::dot(&duu, &dpt) + na::dot(&du, &du);
        let df_uv = na::dot(&duv, &dpt) + na::dot(&du, &dv);
        let df_vv = na::dot(&dvv, &dpt) + na::dot(&dv, &dv);
        let df_vu = na::dot(&dvu, &dpt) + na::dot(&dv, &du);

        let f_u = na::dot(&dpt, &du);
        let f_v = na::dot(&dpt, &dv);

        let f         = Vec2::new(f_u, f_v);
        let mut inv_j = Mat2::new(df_uu, df_uv,
                                  df_vu, df_vv);

        if !inv_j.inv() {
            return None;
        }

        uv = uv - inv_j * f;

        if uv.x < na::zero() || uv.y < na::zero() ||
           uv.x > na::one()  || uv.y > na::one() {
            return None
        }
    }

    Some(b.at(&uv.x, &uv.y, &mut cache))
}
