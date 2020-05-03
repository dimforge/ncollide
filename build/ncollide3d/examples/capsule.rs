use ncollide3d::shape::Capsule;

fn main() {
    let capsule = Capsule::new(0.5f32, 0.75);

    assert!(capsule.half_height() == 0.5);
    assert!(capsule.radius() == 0.75);
}
