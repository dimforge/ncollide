/**
 * Trait of the algorithms executed during the so-called Narrow Phase. The goal
 * of the narrow phase is to determine exactly if two objects collide. If there
 * is collision, it must be able to comptute the exact contact point(s),
 * normal and penetration depth in order to give enough informations to the
 * constraint solver.
 *
 *   - `C`: the type of contact computed.
 *   - `G1`: the type of the first object involved on the collision detection.
 *   - `G2`: the type of the second object involved on the collision detection.
 */
pub trait CollisionDetector<C, G1, G2>
{
  /// Builds a new collision detector. The two objects arguments are hint about
  /// which object are going to be tested.
  fn new(&G1, &G2) -> Self;

  /// Runs the collision detection on two objects. It is assumed that the same
  /// collision detector (the same structure) is always used with the same
  /// pair of object.
  fn update(&mut self, &G1, &G2);

  /// The number of collision detected during the last update.
  fn num_coll(&self) -> uint;

  /// Collects the collisions detected during the last update.
  fn colls(&mut self, &mut ~[@mut C]);
}
