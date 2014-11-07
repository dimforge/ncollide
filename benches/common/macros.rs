#![macro_escape]

macro_rules! bench_free_fn(
    ($name: ident, $function: path $(, $args: ident: $types: ty)*) => {
        #[bench]
        fn $name(bh: &mut Bencher) {
            const LEN: uint = 1 << 13;

            let mut rng = IsaacRng::new_unseeded();

            $(let $args = Vec::from_fn(LEN, |_| random::<$types, _>(&mut rng));)*
            let mut i = 0;

            bh.iter(|| {
                i = (i + 1) & (LEN - 1);

                unsafe {
                    test::black_box($function($(unref($args.unsafe_get(i)),)*))
                }
            })
        }
    }
)

macro_rules! bench_method(
    ($name: ident, $method: ident, $arg: ident: $typ: ty $(, $args: ident: $types: ty)*) => {
        #[bench]
        fn $name(bh: &mut Bencher) {
            const LEN: uint = 1 << 13;

            let mut rng = IsaacRng::new_unseeded();

            let $arg = Vec::from_fn(LEN, |_| random::<$typ, _>(&mut rng));
            $(let $args = Vec::from_fn(LEN, |_| random::<$types, _>(&mut rng));)*
            let mut i = 0;

            bh.iter(|| {
                i = (i + 1) & (LEN - 1);

                unsafe {
                    test::black_box($arg.unsafe_get(i).$method($(unref($args.unsafe_get(i)),)*))
                }
            })
        }
    }
)
