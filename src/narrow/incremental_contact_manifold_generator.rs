use std::num::{One, Zero};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::transformation::Transform;
use narrow::collision_detector::CollisionDetector;
use contact::Contact;

struct ContactWLocals<N, V> {
    local1:  V,
    local2:  V,
    center:  V,
    contact: Contact<N, V>
}

impl<N: DivisionRing + NumCast, V: VectorSpace<N>> ContactWLocals<N, V> {
    fn new_with_contact<G1: Transform<V>,
                        G2: Transform<V>>(
                        contact: Contact<N, V>,
                        g1:      &G1,
                        g2:      &G2)
                        -> ContactWLocals<N, V> {
            ContactWLocals {
                local1: g1.inv_transform(&contact.world1),
                local2: g2.inv_transform(&contact.world2),
                center: (contact.world1 + contact.world2).scalar_div(&(NumCast::from(2.0))),
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
    priv sub_detector: CD
}

impl<CD, N, V> IncrementalContactManifoldGenerator<CD, N, V> {
    /// Creates a new incremental contact manifold generator.
    ///
    /// # Arguments:
    ///   * `cd` - collision detection sub-algorithm used to generate the contact points.
    pub fn new(cd: CD) -> IncrementalContactManifoldGenerator<CD, N, V> {
        IncrementalContactManifoldGenerator {
            contacts:     ~[],
            collector:    ~[],
            sub_detector: cd
        }
    }
}

impl<CD: CollisionDetector<N, V, G1, G2>,
     G1: Transform<V>,
     G2: Transform<V>,
     V: Clone + VectorSpace<N> + Dot<N> + Norm<N> + ApproxEq<N> + Dim,
     N: Clone + DivisionRing + Ord + NumCast>
CollisionDetector<N, V, G1, G2> for IncrementalContactManifoldGenerator<CD, N, V> {
    fn update(&mut self, g1: &G1, g2: &G2) {
        // cleanup existing contacts
        let mut i = 0;
        while i != self.contacts.len() {
            let remove = {
                let c      = &mut self.contacts[i];
                let _2     = One::one::<N>() + One::one();
                let world1 = g1.transform_vec(&c.local1);
                let world2 = g2.transform_vec(&c.local2);

                let dw    = world1 - world2;
                let depth = dw.dot(&c.contact.normal);

                let _tangencial_limit: N = NumCast::from(0.25f64);

                if depth >= Zero::zero() &&
                    (dw - c.contact.normal.scalar_mul(&depth)).sqnorm() <= _tangencial_limit * depth * depth {
                        c.contact.depth = depth;

                        c.contact.world1 = world1;
                        c.contact.world2 = world2;

                        i = i + 1;

                        false
                    }
                else {
                    true
                }
            };

            if remove {
                self.contacts.swap_remove(i);
            }
        }

        // add the new ones
        self.sub_detector.update(g1, g2);

        self.sub_detector.colls(&mut self.collector);

        // remove duplicates
        let _max_num_contact = (Dim::dim::<V>() - 1) * 2;

        for c in self.collector.iter() {
            if self.contacts.len() == _max_num_contact {
                add_reduce_by_variance(self.contacts, c.clone(), g1, g2)
            }
            else {
                self.contacts.push(ContactWLocals::new_with_contact(c.clone(), g1, g2))
            }
        }

        self.collector.clear();
    }

    #[inline]
    fn num_coll(&self) -> uint {
        self.contacts.len()
    }

    #[inline]
    fn colls(&mut self, out_colls: &mut ~[Contact<N, V>]) {
        for c in self.contacts.iter() {
            out_colls.push(c.contact.clone())
        }
    }
}

fn add_reduce_by_variance<N:  DivisionRing + NumCast + Ord,
                          V:  Clone + VectorSpace<N> + Norm<N>,
                          G1: Transform<V>,
                          G2: Transform<V>>(
                          pts:    &mut [ContactWLocals<N, V>],
                          to_add: Contact<N, V>,
                          g1:     &G1,
                          g2:     &G2) {
    let mut argmax = 0;
    let mut varmax = approx_variance(pts, &to_add, 0);

    for i in range(1u, pts.len()) {
        let var = approx_variance(pts, &to_add, i);

        if var > varmax {
            argmax = i;
            varmax = var;
        }
    }

    pts[argmax] = ContactWLocals::new_with_contact(to_add, g1, g2);
}

fn approx_variance<N: DivisionRing + NumCast,
V: Clone + VectorSpace<N> + Norm<N>>(
    pts:       &[ContactWLocals<N, V>],
    to_add:    &Contact<N, V>,
    to_ignore: uint) -> N {
    // first: compute the mean
    let to_add_center = (to_add.world1 + to_add.world2).scalar_div(&NumCast::from(2.0));

    let mut mean = to_add_center.clone();

    for i in range(0u, pts.len()) {
        if i != to_ignore {
            mean = mean + pts[i].center
        }
    }

    mean.scalar_div_inplace(&NumCast::from(pts.len()));

    // compute the sum of variances along all axis
    let mut sum = (to_add_center - mean).sqnorm();

    for i in range(0u, pts.len()) {
        if i != to_ignore {
            sum = sum + (pts[i].center - mean).sqnorm();
        }
    }

    sum
}
