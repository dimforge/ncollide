use na::{self, Isometry2, Vector2};

use ncollide2d::query;
use ncollide2d::shape::{Compound, Cuboid, ShapeHandle};

// Issue #171.
#[test]
fn normal() {
  let c1 = Cuboid::new(Vector2::new(1.0, 1.0));
  let c2 = Cuboid::new(Vector2::new(1.0, 1.0));

  let contact = query::contact(
    &Isometry2::new(Vector2::new(10.5, 10.5), na::zero()),
    &c2,
    &Isometry2::new(Vector2::new(10.0, 10.0), na::zero()),
    &c1,
    1.0,
  );

  assert!(contact.is_some()); // ok
}

// Issue #171.
#[test]
fn compound() {
  let c1 = Cuboid::new(Vector2::new(1.0, 1.0));
  let c2 = Cuboid::new(Vector2::new(1.0, 1.0));

  let shapes = vec![(
    Isometry2::new(Vector2::new(10.0, 10.0), na::zero()),
    ShapeHandle::new(c1),
  )];
  let compound = Compound::new(shapes);

  let contact = query::contact(
    &Isometry2::new(Vector2::new(10.5, 10.5), na::zero()),
    &c2,
    &Isometry2::identity(),
    &compound,
    1.0,
  );

  assert!(contact.is_some()); // FAILED
}
