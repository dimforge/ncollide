extern crate ncollide = "ncollide3df32";

use ncollide::geom::Cone;

fn main() {
    let cone1 = Cone::new(0.5, 0.75);
    let cone2 = Cone::new_with_margin(0.5, 0.75, 0.1);

    assert!(cone1.margin() == 0.04); // default margin
    assert!(cone2.margin() == 0.1);  // user-defined margin
}
