use std::marker::PhantomData;
use std::cell::RefCell;
use math::{Isometry, Point};
use utils::IdAllocator;
use geometry::shape::{AnnotatedPoint, Shape};
use geometry::query::algorithms::Simplex;
use geometry::query::algorithms::gjk::GJKResult;
use geometry::query::contacts_internal;
use geometry::query::{Contact, ContactManifold, ContactPrediction};
use geometry::shape::ConvexPolyface;
use narrow_phase::{ContactDispatcher, ContactGenerator};

/// Persistent collision detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct SupportMapSupportMapContactGenerator<P: Point, M, S> {
    simplex: S,
    contact: GJKResult<Contact<P>, P::Vector>,
    contact_manifold: ContactManifold<P>,
    manifold1: ConvexPolyface<P>,
    manifold2: ConvexPolyface<P>,
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
            contact_manifold: ContactManifold::new(),
            manifold1: ConvexPolyface::new(),
            manifold2: ConvexPolyface::new(),
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
        id_alloc: &mut IdAllocator,
    ) -> bool {
        if let (Some(sma), Some(smb)) = (a.as_support_map(), b.as_support_map()) {
            let initial_direction = match self.contact {
                GJKResult::NoIntersection(ref separator) => Some(separator.clone()),
                GJKResult::Projection(ref contact, _) => Some(contact.normal.unwrap()),
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

            true
        } else {
            false
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        self.contact_manifold.len()
    }

    #[inline]
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<P>>) {
        out.push(&self.contact_manifold)
    }
}
