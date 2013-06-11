/**
 * Trait of broad phase. A broad phase is the first collision detection routine
 * used to remove quickly pairs of objects not colliding. It is assumed to be
 * fast(er than the narrow phase) and must not miss any collision. It can
 * report false positive (collision where there is no).
 *
 *   - `RB`: type of object manipulated by the broad phase.
 */
pub trait BroadPhase<RB>
{
  // FIXME: is '@mut RB' the best? Could using only 'RB' be more flexible
  /// Register an object to be taken in account by the broad phase.
  fn add(&mut self, @mut RB);
  /// Unregister an object from being taken in account by the broad phase.
  fn remove(&mut self, @mut RB);
  /**
   * Gets all the potential collision pairs detected by the broad phase.
   * The output must be carefully formated. Each first object of a collision
   * pair is named a "reference" object:
   *
   *   - all potential collisions with reference objects must be reported
   *   (keeping the same object as reference).
   *   - any collision not reported with a reference object is assumed to have
   *   vanished and can be removed from the collision graph.
   *   - no object should collide with itself.
   */
  fn collision_pairs(&mut self, &[@mut RB]) -> ~[(@mut RB, @mut RB)];
}
