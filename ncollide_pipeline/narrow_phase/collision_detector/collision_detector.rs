use entities::inspection::{Shape, ShapeDesc};
use queries::geometry::Contact;
use math::{Point, Vector};

/// Trait implemented algorithms that compute contact points, normals and penetration depths.
pub trait CollisionDetector<P: Point, M> {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    // FIXME: use ShapeDesc instead of Shape (to avoid useless virtual method calls) ?
    fn update(&mut self,
              dispatcher: &CollisionDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool;

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> usize;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut Vec<Contact<P>>);
}

pub type CollisionAlgorithm<P, M> = Box<CollisionDetector<P, M> + 'static>;

pub trait CollisionDispatcher<P, M> {
    /// Allocate a collision algorithm corresponding to the given pair of shapes.
    fn get_collision_algorithm(&self, a: &ShapeDesc<P, M>, b: &ShapeDesc<P, M>) -> Option<CollisionAlgorithm<P, M>>;
}
