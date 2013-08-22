use nalgebra::traits::dim::Dim;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::vector::{Vec, AlgebraicVec};
use narrow::collision_detector::CollisionDetector;
use contact::Contact;

struct ContactWLocals<N, V> {
    local1:  V,
    local2:  V,
    center:  V,
    contact: Contact<N, V>
}

impl<N: Num + NumCast, V: Vec<N>> ContactWLocals<N, V> {
    fn new_with_contact<M: Transform<V>>(
                        contact: Contact<N, V>,
                        m1:      &M,
                        m2:      &M)
                        -> ContactWLocals<N, V> {
            ContactWLocals {
                local1: m1.inv_transform(&contact.world1),
                local2: m2.inv_transform(&contact.world2),
                center: (contact.world1 + contact.world2) / NumCast::from(2.0),
                contact: contact
            }
        }
}

/// Incremental contact manifold generator wich keep track and update several contacts through
/// multiple consecutive frames. One contact is added per frame until the maximum number of contact
/// is reached. When the maximum number of contact is reached, each time a new contact is created,
/// the new manifold is computed by maximizing the variance along each canonical axis (of the space
/// in which leaves the contacts).
pub struct IncrementalContactManifoldGenerator<CD, N, V> {
    priv contacts:     ~[ContactWLocals<N, V>],
    priv collector:    ~[Contact<N, V>],
    priv prediction:   N,
    priv sub_detector: CD
}

impl<CD, N, V> IncrementalContactManifoldGenerator<CD, N, V> {
    /// Creates a new incremental contact manifold generator.
    ///
    /// # Arguments:
    ///   * `cd` - collision detection sub-algorithm used to generate the contact points.
    pub fn new(prediction: N, cd: CD) -> IncrementalContactManifoldGenerator<CD, N, V> {
        IncrementalContactManifoldGenerator {
            contacts:     ~[],
            collector:    ~[],
            prediction:   prediction,
            sub_detector: cd
        }
    }
}

impl<CD: CollisionDetector<N, V, M, G1, G2>,
     G1,
     G2,
     M: Transform<V>,
     V: Clone + AlgebraicVec<N> + ApproxEq<N>,
     N: Clone + Num + Ord + NumCast + Algebraic>
IncrementalContactManifoldGenerator<CD, N, V> {
    /// Gets a collision from the sub-detector used by this manifold generator. This does not
    /// update the manifold itself.
    pub fn get_sub_collision(&mut self, m1: &M, g1: &G1, m2: &M, g2: &G2) -> Option<Contact<N, V>> {
        self.sub_detector.update(m1, g1, m2, g2);
        self.sub_detector.colls(&mut self.collector);

        let res = if self.collector.len() == 0 {
            None
        }
        else {
            Some(self.collector[0].clone())
        };

        self.collector.clear();

        res
    }

    /// Updates the current manifold by adding one point.
    pub fn add_new_contacts(&mut self, m1: &M, g1: &G1, m2: &M, g2: &G2) {
        // add the new ones
        self.sub_detector.update(m1, g1, m2, g2);

        self.sub_detector.colls(&mut self.collector);

        // remove duplicates
        let _max_num_contact = (Dim::dim::<V>() - 1) * 2;

        for c in self.collector.iter() {
            if self.contacts.len() == _max_num_contact {
                add_reduce_by_variance(self.contacts, c.clone(), m1, m2)
            }
            else {
                self.contacts.push(ContactWLocals::new_with_contact(c.clone(), m1, m2))
            }
        }

        self.collector.clear();
    }

    /// Updates the contacts already existing on this manifold.
    pub fn update_contacts(&mut self, m1: &M, m2: &M) {
        // cleanup existing contacts
        let mut i = 0;
        while i != self.contacts.len() {
            let remove = {
                let c      = &mut self.contacts[i];
                let world1 = m1.transform(&c.local1);
                let world2 = m2.transform(&c.local2);

                let dw    = world1 - world2;
                let depth = dw.dot(&c.contact.normal);

                if depth >= -self.prediction &&
                   (dw - c.contact.normal * depth).sqnorm() <= NumCast::from(0.01f64) {
                        c.contact.depth = depth;

                        c.contact.world1 = world1;
                        c.contact.world2 = world2;

                        false
                    }
                else {
                    true
                }
            };

            if remove {
                self.contacts.swap_remove(i);
            }
            else {
                i = i + 1;
            }
        }
    }
}

impl<CD: CollisionDetector<N, V, M, G1, G2>,
     G1,
     G2,
     M: Transform<V>,
     V: Clone + AlgebraicVec<N> + ApproxEq<N>,
     N: Clone + Num + Ord + NumCast + Algebraic>
CollisionDetector<N, V, M, G1, G2> for IncrementalContactManifoldGenerator<CD, N, V> {
    #[inline]
    fn update(&mut self, m1: &M, g1: &G1, m2: &M, g2: &G2) {
        self.update_contacts(m1, m2);
        self.add_new_contacts(m1, g1, m2, g2);
    }

    #[inline]
    fn num_coll(&self) -> uint {
        self.contacts.len()
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, V>]) {
        for c in self.contacts.iter() {
            out_colls.push(c.contact.clone())
        }
    }

    #[inline]
    fn toi(m1: &M, dir: &V, dist: &N, g1: &G1, m2: &M, g2: &G2) -> Option<N> {
        CollisionDetector::toi::<N, V, M, G1, G2, CD>(m1, dir, dist, g1, m2, g2)
    }

}

fn add_reduce_by_variance<N: Num + NumCast + Algebraic + Ord,
                          V: Clone + AlgebraicVec<N>,
                          M: Transform<V>>(
                          pts:    &mut [ContactWLocals<N, V>],
                          to_add: Contact<N, V>,
                          m1:     &M,
                          m2:     &M) {
    let mut argmax = 0;
    let mut varmax = approx_variance(pts, &to_add, 0);

    for i in range(1u, pts.len()) {
        let var = approx_variance(pts, &to_add, i);

        if var > varmax {
            argmax = i;
            varmax = var;
        }
    }

    pts[argmax] = ContactWLocals::new_with_contact(to_add, m1, m2);
}

fn approx_variance<N: Num + NumCast + Algebraic, V: Clone + AlgebraicVec<N>>(
    pts:       &[ContactWLocals<N, V>],
    to_add:    &Contact<N, V>,
    to_ignore: uint) -> N {
    // first: compute the mean
    let to_add_center = (to_add.world1 + to_add.world2) / NumCast::from(2.0);

    let mut mean = to_add_center.clone();

    for i in range(0u, pts.len()) {
        if i != to_ignore {
            mean = mean + pts[i].center
        }
    }

    mean = mean / NumCast::from(pts.len());

    // compute the sum of variances along all axis
    let mut sum = (to_add_center - mean).sqnorm();

    for i in range(0u, pts.len()) {
        if i != to_ignore {
            sum = sum + (pts[i].center - mean).sqnorm();
        }
    }

    sum
}
