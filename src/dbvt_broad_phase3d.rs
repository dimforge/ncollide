extern crate "nalgebra" as na;
extern crate "ncollide3df32" as ncollide;

use std::rc::Rc;
use std::cell::RefCell;
use na::{Vec3, Iso3};
use ncollide::geom::Ball;
use ncollide::bounding_volume::WithAABB;
use ncollide::broad::{DBVTBroadPhase, NoIdDispatcher, BroadPhase, InterferencesBroadPhase};

fn main() {
    type Shape<'a> = Rc<RefCell<WithAABB<Ball>>>;

    /*
     * Create the objects.
     */
    let pos1 = Iso3::new(Vec3::new(0.0, 0.0, 0.0), na::zero());
    let pos2 = Iso3::new(Vec3::new(0.0, 0.5, 0.0), na::zero());
    let pos3 = Iso3::new(Vec3::new(0.5, 0.0, 0.0), na::zero());
    let pos4 = Iso3::new(Vec3::new(0.5, 0.5, 0.0), na::zero());

    let geom1 = Ball::new(0.5);
    let geom2 = Ball::new(0.5);
    let geom3 = Ball::new(0.5);
    let geom4 = Ball::new(0.5);

    let obj1 = Rc::new(RefCell::new(WithAABB(pos1, geom1)));
    let obj2 = Rc::new(RefCell::new(WithAABB(pos2, geom2)));
    let obj3 = Rc::new(RefCell::new(WithAABB(pos3, geom3)));
    let obj4 = Rc::new(RefCell::new(WithAABB(pos4, geom4)));

    /*
     * Create the broad phase.
     */
    let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher::new();
    let mut bf = DBVTBroadPhase::new(dispatcher, 0.2);

    bf.add(obj1.clone());
    bf.add(obj2.clone());
    bf.add(obj3.clone());
    bf.add(obj4.clone());

    bf.update();

    assert!(bf.num_interferences() == 6);

    // Deactivate everybody.
    bf.deactivate(&obj1);
    bf.deactivate(&obj2);
    bf.deactivate(&obj3);
    bf.deactivate(&obj4);

    // Deactivated bodies do not interfere with each other.
    assert_eq!(bf.num_interferences(), 0);

    // Reactivate everybody.
    bf.activate(&obj1, |_, _, _| { });
    bf.activate(&obj2, |_, _, _| { });
    bf.activate(&obj3, |_, _, _| { });
    bf.activate(&obj4, |_, _, _| { });

    assert!(bf.num_interferences() == 6);

    // Remove two objects.
    bf.remove(&obj1);
    bf.remove(&obj2);

    // Update the broad phase.
    bf.update();

    assert!(bf.num_interferences() == 1)
}
