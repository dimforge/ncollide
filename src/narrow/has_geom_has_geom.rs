use narrow::collision_detector::CollisionDetector;
use geom::has_geom::HasGeom;

struct HasGeomHasGeomCollisionDetector<C, RB1, RB2, G1, G2, SD>
{ sub_detector: SD }

impl<C,
     RB1: HasGeom<G1>,
     RB2: HasGeom<G2>,
     SD:  CollisionDetector<C, G1, G2>,
     G1,
     G2>
CollisionDetector<C, RB1, RB2>
for HasGeomHasGeomCollisionDetector<C, RB1, RB2, G1, G2, SD>
{
  fn new(b1: &RB1, b2: &RB2)
     -> HasGeomHasGeomCollisionDetector<C, RB1, RB2, G1, G2, SD>
  {
    HasGeomHasGeomCollisionDetector {
      sub_detector: CollisionDetector::new::<C, G1, G2, SD>(b1.geom(), b2.geom())
    }
  }

  fn update(&mut self, b1: &RB1, b2: &RB2)
  { self.sub_detector.update(b1.geom(), b2.geom()) }

  fn num_coll(&self) -> uint
  { self.sub_detector.num_coll() }

  fn colls(&mut self, colls: &mut ~[@mut C])
  { self.sub_detector.colls(colls) }
}
