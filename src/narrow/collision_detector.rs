use contact::Contact;

/**
 * Trait of the algorithms executed during the so-called Narrow Phase.
 *
 * The goal of the narrow phase is to determine exactly if two objects collide. If there is
 * collision, it must be able to comptute the exact contact point(s), normal and penetration depth
 * in order to give enough informations to the constraint solver.
 *
 * # Arguments
 * * `N` - the type of the penetration depth.
 * * `V` - the type of the contact normal and contact points.
 * * `G1`- the type of the first object involved on the collision detection.
 * * `G2`- the type of the second object involved on the collision detection.
 */
pub trait CollisionDetector<N, V, M, G1, G2> {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn update(&mut self, &M, &G1, &M, &G2);

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> uint;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut ~[Contact<N, V>]);

    /// Computes the time of impact of two objects.
    ///
    /// # Arguments
    /// * `m1`   - the first object transform.
    /// * `dir`  - the first object displacement direction.
    /// * `dist` - the first object displacement distance.
    /// * `g1`   - the first object.
    /// * `m2`   - the second object transform.
    /// * `g2`   - the second object.
    fn toi(unused_self: Option<Self>, m1: &M, dir: &V, dist: &N, g1: &G1, m2: &M, g2: &G2) -> Option<N>;
}

/// Same as the `CollisionDetector` trait but using dynamic dispatch on the geometries.
pub trait DynamicCollisionDetector<N, V, M> {
    // FIXME: is it possible to avoid the 'dyn_' prefix ?
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn dyn_update(&mut self, &M, &Any, &M, &Any);

    /// The number of collision detected during the last update.
    fn dyn_num_colls(&self) -> uint;

    /// Collects the collisions detected during the last update.
    fn dyn_colls(&self, &mut ~[Contact<N, V>]);
}

impl<T: CollisionDetector<N, V, M, G1, G2>, N, V, M, G1: 'static, G2: 'static>
DynamicCollisionDetector<N, V, M> for T {
    #[inline]
    fn dyn_update(&mut self, m1: &M, g1: &Any, m2: &M, g2: &Any) {
        self.update(
            m1,
            g1.as_ref::<G1>().expect("Invalid geometry."),
            m2,
            g2.as_ref::<G2>().expect("Invalid geometry."))
    }

    #[inline]
    fn dyn_num_colls(&self) -> uint {
        self.num_colls()
    }

    #[inline]
    fn dyn_colls(&self, cs: &mut ~[Contact<N, V>]) {
        self.colls(cs)
    }
}

