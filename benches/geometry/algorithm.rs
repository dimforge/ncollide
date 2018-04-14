use algorithm::{JohnsonSimplex, JohnsonSimplexTemplate};
use algorithms::simplex::Simplex;
use na::{Point3, Vector3};
use test::Bencher;
use test;

#[bench]
fn bench_johnson_simplex(bh: &mut Bencher) {
    let a = Point3::new(-0.5f32, -0.5, -0.5);
    let b = Point3::new(0.0, 0.5, 0.0);
    let c = Point3::new(0.5, -0.5, -0.5);
    let d = Point3::new(0.0, -0.5, -0.5);
    let recursion = JohnsonSimplexTemplate::new(3);

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
    let a = Point3::new(-0.5f32, -0.5, -0.5);
    let b = Point3::new(0.0, 0.5, 0.0);
    let c = Point3::new(0.5, -0.5, -0.5);
    let d = Point3::new(0.0, -0.5, -0.5);
    let _ = JohnsonSimplex::<f64, Point3<f64>, Vector3<f64>>::new_w_tls();

    bh.iter(|| {
        let mut spl = JohnsonSimplex::new_w_tls();

        spl.reset(a);

        spl.add_point(b);
        spl.add_point(c);
        spl.add_point(d);

        test::black_box(spl.project_origin_and_reduce());
    })
}
