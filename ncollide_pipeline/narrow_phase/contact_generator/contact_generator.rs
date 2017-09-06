use geometry::shape::Shape;
use geometry::query::Contact;
use math::Point;

/// Trait implemented algorithms that compute contact points, normals and penetration depths.
pub trait ContactGenerator<P: Point, M> : Sync + Send {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn update(&mut self,
              dispatcher: &ContactDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              prediction: P::Real)
              -> bool;

    /// The number of contacts generated the last update.
    fn num_contacts(&self) -> usize;

    /// Collects the contacts generated during the last update.
    fn contacts(&self, &mut Vec<Contact<P>>);
}

pub type ContactAlgorithm<P, M> = Box<ContactGenerator<P, M> + 'static>;

pub trait ContactDispatcher<P, M> : Sync + Send {
    /// Allocate a collision algorithm corresponding to the given pair of shapes.
    fn get_contact_algorithm(&self, a: &Shape<P, M>, b: &Shape<P, M>) -> Option<ContactAlgorithm<P, M>>;
}
