use std::intrinsics::TypeId;
use std::any::Any;
use na::{Translate, Cross, Rotation};
use {Contact, ContactPrediction};
use shape::Ball;
use pipeline::narrow_phase::{CollisionDetector, OneShotContactManifoldGenerator};
use math::{Scalar, Point, Vector, Isometry};

/// Generates a contact manifold from an existing single-contact generator.
pub fn generate_contact_manifold<N, P, V, AV, M, G1, G2>(m1: &Isometry<N>, g1: &G1,
                                                         m2: &Isometry<N>, g2: &G2,
                                                         prediction: ContactPrediction<N>,
                                                         generator: fn(&Isometry<N>, &G1, &Isometry<N>, &G2, N) -> Option<Contact<N, P, V>>,
                                                         out: &mut Vec<Contact<N, P, V>>)
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vector<N> + Translate<P> + Cross<AV>,
          AV: Vector<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          G1: Any,
          G2: Any {
    // Do not try to generate a manifold for balls.
    if g1.get_type_id() == TypeId::of::<Ball<N>>() || g2.get_type_id() == TypeId::of::<Ball<N>>() {
        match generator(m1, g1, m2, g2, prediction) {
            Some(c) => out.push(c),
            None    => { }
        }
    }
    else {
        let one_contact_generator = AdHocContactGenerator::new(generator, prediction);
        let mut manifold_generator = OneShotContactManifoldGenerator::new(prediction, one_contact_generator);

        manifold_generator.update(m1, g1, m2, g2);
        manifold_generator.colls(out);
    }
}

struct AdHocContactGenerator<N, P, V, M, G1, G2> {
    generator:  fn(&Isometry<N>, &G1, &Isometry<N>, &G2, N) -> Option<Contact<N, P, V>>,
    prediction: N,
    contact:    Option<Contact<N, P, V>>
}

impl<N, P, V, M, G1, G2> AdHocContactGenerator<N, P, V, M, G1, G2> {
    pub fn new(generator: fn(&Isometry<N>, &G1, &Isometry<N>, &G2, N) -> Option<Contact<N, P, V>>, prediction: N)
               -> AdHocContactGenerator<N, P, V, M, G1, G2> {
        AdHocContactGenerator {
            generator:  generator,
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N, P, V, M, G1, G2> CollisionDetector<N, P, V, M, G1, G2> for AdHocContactGenerator<N, P, V, M, G1, G2>
    where N: Clone,
          P: Clone,
          V: Clone {
    fn update(&mut self, m1: &Isometry<N>, g1: &G1, m2: &Isometry<N>, g2: &G2) {
        self.contact = (self.generator)(m1, g1, m2, g2, self.prediction);
    }

    #[inline]
    fn num_colls(&self) -> usize {
        if self.contact.is_some() {
            1
        }
        else {
            0
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => { }
        }
    }
}
