use std::rand;
use sync::{Arc, RWLock};
use nalgebra::na;
use nalgebra::na::{Vec2, Mat2, Translation, Inv, Norm};
use math::{Scalar, Vect, Matrix};
use geom::{Ball, BezierSurface};
use narrow::CollisionDetector;
use contact::Contact;
use narrow::surface_selector::SurfaceSelector;
use narrow::surface_subdivision_tree::{SurfaceSubdivisionTreeRef, SurfaceSubdivisionTree, SurfaceSubdivisionTreeCache};

/// Collision detector between a ball and a bezier surface.
pub struct BallBezierSurface<S, D> {
    cache:      Arc<RWLock<SurfaceSubdivisionTreeCache<D>>>,
    tree:       Option<SurfaceSubdivisionTreeRef<D>>, // Keep this for automatic registration on the cache.
    selector:   S,
    prediction: Scalar,
    contacts:   Vec<Contact>,
    points:     Vec<Vect>,
    timestamp:  uint
}

#[unsafe_destructor]
impl<S, D> Drop for BallBezierSurface<S, D> {
    fn drop(&mut self) {
        // XXX: we need to decrement the refcounts here.
    }
}

impl<S: Clone, D: Send> Clone for BallBezierSurface<S, D> {
    fn clone(&self) -> BallBezierSurface<S, D> {
        BallBezierSurface {
            cache:      self.cache.clone(),
            tree:       self.tree.clone(),
            selector:   self.selector.clone(),
            prediction: self.prediction.clone(),
            contacts:   self.contacts.clone(),
            points:     self.points.clone(),
            timestamp:  self.timestamp.clone()
        }
    }
}

impl<S: SurfaceSelector<D>, D> BallBezierSurface<S, D> {
    /// Creates a new collision detector with the given prediction margin.
    pub fn new(selector:   S,
               prediction: Scalar,
               cache:      Arc<RWLock<SurfaceSubdivisionTreeCache<D>>>)
               -> BallBezierSurface<S, D> {
        BallBezierSurface {
            cache:      cache,
            tree:       None,
            selector:   selector,
            prediction: prediction,
            contacts:   Vec::new(),
            points:     Vec::new(),
            timestamp:  0 // 0 because the tree starts at 1.
        }
    }
}

impl<S: SurfaceSelector<D>, D: Send + Share>
CollisionDetector<Ball, BezierSurface> for BallBezierSurface<S, D> {
    fn update(&mut self, ma: &Matrix, a: &Ball, mb: &Matrix, b: &BezierSurface) {
        self.points.clear();
        self.contacts.clear();
        self.selector.set_max_lmd(a.radius() + self.prediction);

        /*
        let renew_tree = match self.tree {
            None           => true,
            Some(ref tree) => !tree.is_the_subdivision_tree_of(b)
        };

        if renew_tree {
            let tree = SurfaceSubdivisionTreeCache::find_or_insert_with(
                           &mut self.cache.clone(), // FIXME: too bad we have to clone here…
                           b,
                           || self.selector.create_test_data(b));
            self.tree = Some(tree);
        }
        */


        let max_depth = 15;
        let niter     = 10;
        let pt        = na::inv_transform(mb, &ma.translation());

        closest_points(&pt, b, niter, &mut self.selector, max_depth, &mut self.points);

        /*
        let tree = self.tree.as_ref().unwrap();
        let timestamp = tree.read().timestamp();

        if timestamp != self.timestamp { // no need to cleanup the tree
            self.timestamp = timestamp;
        }
        else {
            self.timestamp = cleanup_subdivision_tree(tree.deref(), self.timestamp);
        }

        closest_points_with_subdivision_tree(&pt, tree.deref(), &mut self.selector, self.timestamp,
                                             max_depth, niter, &mut self.points);
        */


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
pub struct BezierSurfaceBall<S, D> {
    detector: BallBezierSurface<S, D>
}

impl<S: Clone, D: Send> Clone for BezierSurfaceBall<S, D> {
    fn clone(&self) -> BezierSurfaceBall<S, D> {
        BezierSurfaceBall {
            detector: self.detector.clone()
        }
    }
}

impl<S: SurfaceSelector<D>, D> BezierSurfaceBall<S, D> {
    /// Creates a new collision detector with the given prediction margin.
    pub fn new(selector:   S,
               prediction: Scalar,
               cache:      Arc<RWLock<SurfaceSubdivisionTreeCache<D>>>)
               -> BezierSurfaceBall<S, D> {
        BezierSurfaceBall {
            detector: BallBezierSurface::new(selector, prediction, cache)
        }
    }
}

impl<S: SurfaceSelector<D>, D: Send + Share>
CollisionDetector<BezierSurface, Ball> for BezierSurfaceBall<S, D> {
    fn update(&mut self, ma: &Matrix, a: &BezierSurface, mb: &Matrix, b: &Ball) {
        self.detector.update(mb, b, ma, a);

        for c in self.detector.contacts.mut_iter() {
            c.flip();
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.detector.num_colls()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact>) {
        self.detector.colls(out_colls)
    }

    #[inline]
    fn toi(_: Option<BezierSurfaceBall<S, D>>, _: &Matrix, _: &Vect, _: &Scalar, _: &BezierSurface,
           _: &Matrix, _: &Ball) -> Option<Scalar> {
        None
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
pub fn closest_points<S: SurfaceSelector<D>, D>(pt:        &Vect,
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

// FIXME: too bad this cannot be safely exposed to the user (because of the need to pass a valid
// timestamp).
#[allow(dead_code)] // FIXME: this might be useful in the future, it the subdivision tree becomes handy.
fn closest_points_with_subdivision_tree<S: SurfaceSelector<D>, D: Send + Share>(
                                        pt:             &Vect,
                                        to_visit:       &Arc<RWLock<SurfaceSubdivisionTree<D>>>,
                                        selector:       &mut S,
                                        curr_timestamp: uint,
                                        max_depth:      uint,
                                        niter:          uint,
                                        out:            &mut Vec<Vect>) {
    let can_go_down;
    let accepted;
    let mark_as_accepted;

    /*
     * See if we have a solution, or if we have to drop this node's children.
     */
    {
        let r_to_visit = to_visit.read();
        let surface    = r_to_visit.surface();
        let data       = r_to_visit.data();

        if selector.may_contain_a_closest_point(pt, surface, data) {
            if max_depth == 0 || selector.is_flat(surface, data) {
                match closest_point(pt, surface, niter) {
                    Some(pt) => out.push(pt),
                    None     => { }
                }

                // NOTE: we don't care to mark the leaf as accepted here, since we know its parent
                // is accepted anyway (thus, it will not be removed).
                return;
            }

            accepted         = true;
            mark_as_accepted = r_to_visit.timestamp() != curr_timestamp;
        }
        else {
            // otherwise, we have to mark the current subsurface as rejected.
            accepted         = false;
            mark_as_accepted = false;
        }

        /*
         * We may have a solution, and we have to go down.
         * First, we have to ensure that the current node has two children to recurse on.
         */
        can_go_down = r_to_visit.has_left_child(); // proper tree
    }

    if accepted {
        // it is accepted but not yet marked as such.
        if mark_as_accepted {
            // NOTE: we do not care if multiple task set the timestamp.
            to_visit.write().set_timestamp(curr_timestamp); // FIXME: any way to factor this lock this the one bellow?
        }

        if !can_go_down {
            // At this point, we *might* have to subdivide the tree.
            // This is a *might* because somebody might have created the child in-between the read-lock
            // and the write-lock.
            // Therefore, we have to re-test once the read-write-lock is taken.

            // >>>> get a read-write lock on the node.
            let mut w_to_visit = to_visit.write();

            if !w_to_visit.has_left_child() { // proper tree
                // _Now_ we are sure that we need to subdivide (and we will the only one doing so).

                let mut left  = BezierSurface::new_with_degrees(0, 0);
                let mut right = BezierSurface::new_with_degrees(0, 0);

                if max_depth % 2 == 0 {
                    w_to_visit.surface().subdivide_u(&na::cast(0.5), &mut left, &mut right);
                }
                else {
                    w_to_visit.surface().subdivide_v(&na::cast(0.5), &mut left, &mut right);
                }

                let ldata = selector.create_test_data(&left);
                let rdata = selector.create_test_data(&right);

                let lchild = SurfaceSubdivisionTree::new_orphan(left, ldata, w_to_visit.timestamp());
                let rchild = SurfaceSubdivisionTree::new_orphan(right, rdata, w_to_visit.timestamp());

                w_to_visit.set_right_child(rchild);
                w_to_visit.set_left_child(lchild);
            }
            // <<<< end of read-write lock on the node.
        }

        // We can safely recurse now, and we are guarenteed to have two children.

        let rchild;
        let lchild;

        // >>>>>>>> get a read-only lock on the node.
        {
            let r_to_visit = to_visit.read();
            rchild         = r_to_visit.right_child().expect("Internal error: no right child.");
            lchild         = r_to_visit.left_child().expect("Internal error: no left child.");
        }
        // <<<<<<<< end of read-only lock on the node.

        if rand::random() {
            closest_points_with_subdivision_tree(pt, &rchild, selector, curr_timestamp, max_depth - 1, niter, out);
            closest_points_with_subdivision_tree(pt, &lchild, selector, curr_timestamp, max_depth - 1, niter, out);
        }
        else {
            closest_points_with_subdivision_tree(pt, &lchild, selector, curr_timestamp, max_depth - 1, niter, out);
            closest_points_with_subdivision_tree(pt, &rchild, selector, curr_timestamp, max_depth - 1, niter, out);
        }
    }
}

#[allow(dead_code)] // FIXME: this might be useful in the future, it the subdivision tree becomes handy.
fn cleanup_subdivision_tree<D: Send + Share>(root:            &Arc<RWLock<SurfaceSubdivisionTree<D>>>,
                                             valid_timestamp: uint)
                                             -> uint {
    // NOTE: we are keeping a read-write-lock on the root, to make sure we are alone during the
    // cleanup (this might not be the best solution, but, at least, this is the easiest.
    let mut w_root = root.write();

    // If this test is false, somebody already updated the timestamp before.
    if w_root.timestamp() == valid_timestamp {
        if w_root.has_left_child() { // proper tree
            do_cleanup_subdivision_tree(w_root.right_child_ref().unwrap(), valid_timestamp);
            do_cleanup_subdivision_tree(w_root.left_child_ref().unwrap(),  valid_timestamp);
        }

        let new_timestamp = valid_timestamp + 1;

        w_root.set_timestamp(new_timestamp);

        new_timestamp
    }
    else {
        valid_timestamp
    }
}

#[allow(dead_code)] // FIXME: this might be useful in the future, it the subdivision tree becomes handy.
fn do_cleanup_subdivision_tree<D: Send + Share>(tree: &Arc<RWLock<SurfaceSubdivisionTree<D>>>, valid_timestamp: uint) {
    // >>>>> Read-only lock on `tree`.
    {
        let r_tree = tree.read();

        if r_tree.timestamp() == valid_timestamp {
            // The children of this node are valid: recurse.
            // NOTE: we keep the lock on the tree while we recurse (dont want other people to mess up
            // with the timestamps in the meantime).
            if r_tree.has_left_child() { // proper tree
                do_cleanup_subdivision_tree(r_tree.right_child_ref().unwrap(), valid_timestamp);
                do_cleanup_subdivision_tree(r_tree.left_child_ref().unwrap(), valid_timestamp)
            }

            return;
        }
    }
    // <<<<< End of ead-only lock on `tree`.

    // This node must have its children executed.
    // Heh, life is not always pretty =(
    let mut w_tree = tree.write();
    w_tree.remove_right_child();
    w_tree.remove_left_child();
    w_tree.set_timestamp(valid_timestamp); // to prevent overflow-related problems.
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

        // we allow smalls steps outside of the domain boundaries.
        if uv.x < na::cast(-1.0) || uv.y < na::cast(-1.0) ||
           uv.x > na::cast(2.0)  || uv.y > na::cast(2.0) {
                return None
        }
    }

    if uv.x < na::zero() || uv.y < na::zero() ||
       uv.x > na::one()  || uv.y > na::one() {
        return None
    }

    Some(b.at(&uv.x, &uv.y, &mut cache))
}

#[cfg(test,dim3,f32)]
mod test {
    use std::num::Bounded;
    use nalgebra::na::Vec3;
    use nalgebra::na;
    use geom::BezierSurface;
    use narrow::surface_selector::HyperPlaneSurfaceSelector;

    #[test]
    fn test_boundary_convergence_1() {
        // NOTE: this failed if the hyperplane rejected dot projectude that are == 0.0
        let control_points = vec!(
            Vec3::new(-20.0, -00.0, 020.0), Vec3::new(-10.0, -00.0, 020.0), Vec3::new(00.0, -00.0, 020.0), Vec3::new(10.0, -00.0, 020.0), Vec3::new(20.0, -00.0, 020.0),
            Vec3::new(-20.0, -00.0, 010.0), Vec3::new(-10.0, -00.0, 010.0), Vec3::new(00.0, 000.0, 010.0), Vec3::new(10.0, -00.0, 010.0), Vec3::new(20.0, -00.0, 010.0),
            Vec3::new(-20.0, -00.0, 000.0), Vec3::new(-10.0, -00.0, 000.0), Vec3::new(00.0, 000.0, 000.0), Vec3::new(10.0, -00.0, 000.0), Vec3::new(20.0, -00.0, 000.0),
            Vec3::new(-20.0, -00.0, -10.0), Vec3::new(-10.0, -00.0, -10.0), Vec3::new(00.0, 000.0, -10.0), Vec3::new(10.0, -00.0, -10.0), Vec3::new(20.0, -00.0, -10.0),
            Vec3::new(-20.0, -00.0, -20.0), Vec3::new(-10.0, -00.0, -20.0), Vec3::new(00.0, -00.0, -20.0), Vec3::new(10.0, -00.0, -20.0), Vec3::new(20.0, -00.0, -20.0));

        let pt       = Vec3::new(-2.5 * 0.5 * 0.5, 2.5 * 0.5, -2.5 * 0.5 * 0.5);
        let bezier   = BezierSurface::new(control_points, 5, 5);

        let mut selector = HyperPlaneSurfaceSelector::new(Bounded::max_value());
        let mut pts      = Vec::new();

        super::closest_points(&pt, &bezier, 5, &mut selector, 15, &mut pts);

        assert!(pts.len() >= 1);
        assert!(na::approx_eq(pts.get(0), &Vec3::new(-2.5 * 0.5 * 0.5, 0.0, -2.5 * 0.5 * 0.5)));
    }

    #[test]
    fn test_boundary_convergence_2() {
        let control_points = vec!(
            Vec3::new(-20.0, -00.0, 020.0), Vec3::new(-10.0, -00.0, 020.0), Vec3::new(00.0, -00.0, 020.0), Vec3::new(10.0, -00.0, 020.0), Vec3::new(20.0, -00.0, 020.0),
            Vec3::new(-20.0, -00.0, 010.0), Vec3::new(-10.0, -40.0, 010.0), Vec3::new(00.0, 040.0, 010.0), Vec3::new(10.0, -40.0, 010.0), Vec3::new(20.0, -00.0, 010.0),
            Vec3::new(-20.0, -00.0, 000.0), Vec3::new(-10.0, -40.0, 000.0), Vec3::new(00.0, 040.0, 000.0), Vec3::new(10.0, -40.0, 000.0), Vec3::new(20.0, -00.0, 000.0),
            Vec3::new(-20.0, -00.0, -10.0), Vec3::new(-10.0, -40.0, -10.0), Vec3::new(00.0, 040.0, -10.0), Vec3::new(10.0, -40.0, -10.0), Vec3::new(20.0, -00.0, -10.0),
            Vec3::new(-20.0, -00.0, -20.0), Vec3::new(-10.0, -00.0, -20.0), Vec3::new(00.0, -00.0, -20.0), Vec3::new(10.0, -00.0, -20.0), Vec3::new(20.0, -00.0, -20.0));

        let pt       = Vec3::new(-9.018015, -7.855951, -0.056726);
        let bezier   = BezierSurface::new(control_points, 5, 5);

        let mut selector = HyperPlaneSurfaceSelector::new(0.6);
        let mut pts      = Vec::new();

        super::closest_points(&pt, &bezier, 5, &mut selector, 15, &mut pts);

        assert!(pts.len() >= 1);
    }
}
