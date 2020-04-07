extern crate ncollide3d;

use ncollide3d::pipeline::CollisionGroups;

fn main() {
    let a = CollisionGroups::new()
        .with_membership(&[1, 3, 6])
        .with_whitelist(&[6, 7])
        .with_blacklist(&[1]);
    let b = CollisionGroups::new()
        .with_membership(&[1, 3, 7])
        .with_whitelist(&[3, 7]);
    let c = CollisionGroups::new()
        .with_membership(&[6, 9])
        .with_whitelist(&[3, 7]);

    assert!(!a.can_interact_with_groups(&b));
    assert!(!b.can_interact_with_groups(&c));
    assert!(a.can_interact_with_groups(&c));
}
