use std::marker::PhantomData;
use math::{Isometry, Point};
use geometry::shape::{AnnotatedPoint, Shape};
use geometry::query::algorithms::Simplex;
use geometry::query::algorithms::gjk::GJKResult;
use geometry::query::contacts_internal;
use geometry::query::{Contact, ContactPrediction};
use narrow_phase::{ContactDispatcher, ContactGenerator};

/// Persistent collision detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct SupportMapSupportMapContactGenerator<P: Point, M, S> {
    simplex: S,
    contact: GJKResult<Contact<P>, P::Vector>,
    contact_manifold: Vec<Contact<P>>,
    manifold1: Vec<P>,
    manifold2: Vec<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P, M, S> SupportMapSupportMapContactGenerator<P, M, S>
where
    P: Point,
    S: Simplex<AnnotatedPoint<P>>,
{
    /// Creates a new persistant collision detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(simplex: S) -> SupportMapSupportMapContactGenerator<P, M, S> {
        SupportMapSupportMapContactGenerator {
            simplex: simplex,
            contact: GJKResult::Intersection,
            contact_manifold: Vec::new(),
            manifold1: Vec::new(),
            manifold2: Vec::new(),
            mat_type: PhantomData,
        }
    }
}

impl<P, M, S> ContactGenerator<P, M> for SupportMapSupportMapContactGenerator<P, M, S>
where
    P: Point,
    M: Isometry<P>,
    S: Simplex<AnnotatedPoint<P>>,
{
    #[inline]
    fn update(
        &mut self,
        _: &ContactDispatcher<P, M>,
        ma: &M,
        a: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
    ) -> bool {
        if let (Some(sma), Some(smb)) = (a.as_support_map(), b.as_support_map()) {
            let initial_direction = match self.contact {
                GJKResult::NoIntersection(ref separator) => Some(separator.clone()),
                GJKResult::Projection(ref contact) => Some(contact.normal.unwrap()),
                GJKResult::Intersection => None,
                GJKResult::Proximity(_) => unreachable!(),
            };

            self.contact = contacts_internal::support_map_against_support_map_with_params(
                ma,
                sma,
                mb,
                smb,
                prediction.linear,
                &mut self.simplex,
                initial_direction,
            );

            // // Generate a contact manifold.
            // self.manifold1.clear();
            // self.manifold2.clear();

            // match self.contact {
            //     GJKResult::Projection(ref contact) => {
            //         sma.support_area_toward(ma, &contact.normal, predicton.angular1, &mut self.manifold1);
            //         if self.manifold1.len() > 1 {
            //             smb.support_area_toward(mb, &contact.normal, prediction.angular2, &mut self.manifold2);
            //             if self.manifold2.len() > 1 {
            //                 // Generate contacts at the vertices of the polyhedral contact area.
            //                 utils::overlay_ccw_areas(&self.manifold1[..], &self.manifold2[..], &mut self.overlay)
            //             }
            //         }

            //     },
            //     _ => { }
            // }

            true
        } else {
            false
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        match self.contact {
            GJKResult::Projection(_) => 1,
            _ => 0,
        }
    }

    #[inline]
    fn contacts(&self, out_contacts: &mut Vec<Contact<P>>) {
        match self.contact {
            GJKResult::Projection(ref c) => out_contacts.push(c.clone()),
            _ => (),
        }
    }
}
