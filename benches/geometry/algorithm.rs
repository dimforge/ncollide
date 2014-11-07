use geometry::algorithm::{JohnsonSimplex, RecursionTemplate};
use geometry::algorithms::simplex::Simplex;
use na::{Pnt3, Vec3};
use test::Bencher;
use test;

#[bench]
fn bench_johnson_simplex(bh: &mut Bencher) {
    let a = Pnt3::new(-0.5f32, -0.5, -0.5);
    let b = Pnt3::new(0.0, 0.5, 0.0);
    let c = Pnt3::new(0.5, -0.5, -0.5);
    let d = Pnt3::new(0.0, -0.5, -0.5);
    let recursion = RecursionTemplate::new(3);

    bh.iter(|| {
        let mut spl = JohnsonSimplex::new(recursion.clone());

        spl.reset(a);

        spl.add_point(b);
        spl.add_point(c);
        spl.add_point(d);

        test::black_box(spl.project_origin_and_reduce());
    })
}

#[bench]
fn bench_johnson_simplex_tls(bh: &mut Bencher) {
    let a = Pnt3::new(-0.5f32, -0.5, -0.5);
    let b = Pnt3::new(0.0, 0.5, 0.0);
    let c = Pnt3::new(0.5, -0.5, -0.5);
    let d = Pnt3::new(0.0, -0.5, -0.5);
    let _ = JohnsonSimplex::<f64, Pnt3<f64>, Vec3<f64>>::new_w_tls();

    bh.iter(|| {
        let mut spl = JohnsonSimplex::new_w_tls();

        spl.reset(a);

        spl.add_point(b);
        spl.add_point(c);
        spl.add_point(d);

        test::black_box(spl.project_origin_and_reduce());
    })
}
