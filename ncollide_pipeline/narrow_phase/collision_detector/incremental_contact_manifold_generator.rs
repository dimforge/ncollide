use std::marker::PhantomData;
use na::Transform;
use na;
use math::{Point, Vector};
use queries::geometry::Contact;
use entities::inspection::Shape;
use narrow_phase::{CollisionDetector, CollisionDispatcher};


#[derive(RustcEncodable, RustcDecodable, Clone)]
struct ContactWLocals<P: Point> {
    local1:  P,
    local2:  P,
    center:  P,
    contact: Contact<P>
}

impl<P> ContactWLocals<P>
    where P: Point {
    fn new_with_contact<M: Transform<P>>(contact: Contact<P>, m1: &M, m2: &M) -> ContactWLocals<P> {
            ContactWLocals {
                local1: m1.inverse_transform(&contact.world1),
                local2: m2.inverse_transform(&contact.world2),
                center: na::center(&contact.world1, &contact.world2),
                contact: contact
            }
        }
}

/// Contact manifold generator which keeps track of several contacts.
///
/// One contact is added per update until the maximum number of contact is reached. When the
/// maximum number of contact is reached, each time a new contact is created, the new manifold is
/// computed by maximizing the variance along each canonical axis (of the space in which leaves the
/// contacts).
#[derive(RustcEncodable, RustcDecodable, Clone)]
pub struct IncrementalContactManifoldGenerator<P: Point, M, CD> {
    contacts:     Vec<ContactWLocals<P>>, // FIXME: replace by a vec slice to avoid allocations ?
    collector:    Vec<Contact<P>>,        // FIXME: replace by a vec slice to avoid allocations ?
    sub_detector: CD,
    _matrix:      PhantomData<M>
}

impl<P, M, CD> IncrementalContactManifoldGenerator<P, M, CD>
    where P:  Point,
          CD: CollisionDetector<P, M> {
    /// Creates a new incremental contact manifold generator.
    ///
    /// # Arguments:
    /// * `cd` - collision detection sub-algorithm used to generate the contact points.
    pub fn new(cd: CD) -> IncrementalContactManifoldGenerator<P, M, CD> {
        IncrementalContactManifoldGenerator {
            contacts:     Vec::new(),
            collector:    Vec::new(),
            sub_detector: cd,
            _matrix:      PhantomData
        }
    }
}

impl<P, M, CD> IncrementalContactManifoldGenerator<P, M, CD>
    where P:  Point,
          M:  Transform<P>,
          CD: CollisionDetector<P, M> {
    /// Gets a collision from the sub-detector used by this manifold generator. This does not
    /// update the manifold itself.
    pub fn get_sub_collision(&mut self,
                             d:          &CollisionDispatcher<P, M>,
                             m1:         &M,
                             g1:         &Shape<P, M>,
                             m2:         &M,
                             g2:         &Shape<P, M>,
                             prediction: <P::Vect as Vector>::Scalar)
                             -> Option<Option<Contact<P>>> {
        if !self.sub_detector.update(d, m1, g1, m2, g2, prediction) {
            None
        }
        else {
            self.sub_detector.colls(&mut self.collector);

            let res = if self.collector.len() == 0 {
                Some(None)
            }
            else {
                Some(Some(self.collector[0].clone()))
            };

            self.collector.clear();

            res
        }
    }

    /// Updates the current manifold by adding one point.
    pub fn add_new_contacts(&mut self,
                            d:  &CollisionDispatcher<P, M>,
                            m1: &M,
                            g1: &Shape<P, M>,
                            m2: &M,
                            g2: &Shape<P, M>,
                            prediction: <P::Vect as Vector>::Scalar)
                            -> bool {
        // add the new ones
        if !self.sub_detector.update(d, m1, g1, m2, g2, prediction) {
            false
        }
        else {
            self.sub_detector.colls(&mut self.collector);

            // remove duplicates
            let _max_num_contact = (na::dimension::<P>() - 1) * 2;

            for c in self.collector.iter() {
                if self.contacts.len() == _max_num_contact {
                    add_reduce_by_variance(&mut self.contacts[..], c.clone(), m1, m2)
                }
                else {
                    self.contacts.push(ContactWLocals::new_with_contact(c.clone(), m1, m2))
                }
            }

            self.collector.clear();

            true
        }
    }

    /// Updates the contacts already existing on this manifold.
    pub fn update_contacts(&mut self, m1: &M, m2: &M, prediction: <P::Vect as Vector>::Scalar) {
        // cleanup existing contacts
        let mut i = 0;
        while i != self.contacts.len() {
            let remove = {
                let c      = &mut self.contacts[i];
                let world1 = m1.transform(&c.local1);
                let world2 = m2.transform(&c.local2);

                let dw    = world1 - world2;
                let depth = na::dot(&dw, &c.contact.normal);

                if depth >= -prediction &&
                   na::norm_squared(&(dw - c.contact.normal * depth)) <= na::cast(0.01f64) {
                   c.contact.depth  = depth;
                   c.contact.world1 = world1;
                   c.contact.world2 = world2;

                   false
                }
                else {
                    true
                }
            };

            if remove {
                let _ = self.contacts.swap_remove(i);
            }
            else {
                i = i + 1;
            }
        }
    }
}

impl<P, M, CD> CollisionDetector<P, M> for IncrementalContactManifoldGenerator<P, M, CD>
    where P:  Point,
          M:  Transform<P>,
          CD: CollisionDetector<P, M> {
    #[inline]
    fn update(&mut self,
              d:          &CollisionDispatcher<P, M>,
              m1:         &M,
              g1:         &Shape<P, M>,
              m2:         &M,
              g2:         &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        self.update_contacts(m1, m2, prediction);
        self.add_new_contacts(d, m1, g1, m2, g2, prediction)
    }

    #[inline]
    fn num_colls(&self) -> usize {
        self.contacts.len()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<P>>) {
        for c in self.contacts.iter() {
            out_colls.push(c.contact.clone())
        }
    }
}

fn add_reduce_by_variance<P, M>(pts: &mut [ContactWLocals<P>], to_add: Contact<P>, m1: &M, m2: &M)
    where P:  Point,
          M:  Transform<P> {
    let mut argmax = 0;
    let mut varmax = approx_variance(pts, &to_add, 0);

    for i in 1usize .. pts.len() {
        let var = approx_variance(pts, &to_add, i);

        if var > varmax {
            argmax = i;
            varmax = var;
        }
    }

    pts[argmax] = ContactWLocals::new_with_contact(to_add, m1, m2);
}

fn approx_variance<P>(pts: &[ContactWLocals<P>], to_add: &Contact<P>, to_ignore: usize) -> <P::Vect as Vector>::Scalar
    where P: Point {
    // first: compute the mean
    let to_add_center = na::center(&to_add.world1, &to_add.world2);

    let mut mean = to_add_center.clone();

    for i in 0usize .. pts.len() {
        if i != to_ignore {
            mean = mean + *pts[i].center.as_vector()
        }
    }

    let divisor: f64 = 1.0f64 / (pts.len() as f64);
    mean = mean * na::cast(divisor);

    // compute the sum of variances along all axis
    let mut sum = na::norm_squared(&(to_add_center - mean));

    for i in 0usize .. pts.len() {
        if i != to_ignore {
            sum = sum + na::norm_squared(&(pts[i].center - mean));
        }
    }

    sum
}
