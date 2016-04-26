extern crate ncollide;

use ncollide::world::CollisionGroups;

fn main() {
    let mut a = CollisionGroups::new();
    let mut b = CollisionGroups::new();
    let mut c = CollisionGroups::new();

    a.set_membership(&[1, 3, 6]);
    a.set_whitelist(&[6, 7]);
    a.set_blacklist(&[1]);

    b.set_membership(&[1, 3, 7]);
    b.set_whitelist(&[3, 7]);

    c.set_membership(&[6, 9]);
    c.set_whitelist(&[3, 7]);

    assert!(!a.can_interact_with_groups(&b));
    assert!(!b.can_interact_with_groups(&c));
    assert!(a.can_interact_with_groups(&c));
}
