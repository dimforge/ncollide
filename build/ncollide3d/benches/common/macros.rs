macro_rules! bench_free_fn (
    ($name: ident, $function: path, $($args: ident: $types: ty),*) => {
        bench_free_fn_gen!($name, $function, $($args: $types = generate),*);
    }
);

macro_rules! bench_method (
    ($name: ident, $method: ident, $arg: ident: $typ: ty, $($args: ident: $types: ty),*) => {
        bench_method_gen!($name, $method, $arg: $typ = generate, $($args: $types = generate),*);
    };
    ($name: ident, $method: ident: $tres: ty, $arg: ident: $typ: ty, $($args: ident: $types: ty),*) => {
        bench_method_gen!($name, $method: $tres, $arg: $typ = generate, $($args: $types = generate),*);
    };
);

macro_rules! bench_free_fn_gen (
    ($name: ident, $function: path, $($args: ident: $types: ty = $gens: path),*) => {
        #[bench]
        fn $name(bh: &mut Bencher) {
            const LEN: usize = 1 << 7;

            let mut rng: IsaacRng = SeedableRng::seed_from_u64(0);

            $(let $args: Vec<$types> = (0usize .. LEN).map(|_| $gens(&mut rng)).collect();)*
            let mut i = 0;

            bh.iter(|| {
                i = (i + 1) & (LEN - 1);

                unsafe {
                    test::black_box($function($(unref($args.get_unchecked(i)),)*))
                }
            });
        }
    }
);

macro_rules! bench_method_gen (
    ($name: ident, $method: ident, $arg: ident: $typ: ty = $gen: path, $($args: ident: $types: ty = $gens: path),*) => {
        #[bench]
        fn $name(bh: &mut Bencher) {
            const LEN: usize = 1 << 7;

            let mut rng: IsaacRng = SeedableRng::seed_from_u64(0);

            let $arg: Vec<$typ> = (0usize .. LEN).map(|_| $gen(&mut rng)).collect();
            $(let $args: Vec<$types> = (0usize .. LEN).map(|_| $gens(&mut rng)).collect();)*
            let mut i = 0;

            bh.iter(|| {
                i = (i + 1) & (LEN - 1);

                unsafe {
                    test::black_box($arg.get_unchecked(i).$method($(unref($args.get_unchecked(i)),)*))
                }
            })
        }
    };
    ($name: ident, $method: ident: $tres: ty, $arg: ident: $typ: ty = $gen: path, $($args: ident: $types: ty = $gens: path),*) => {
        #[bench]
        fn $name(bh: &mut Bencher) {
            const LEN: usize = 1 << 7;

            let mut rng: IsaacRng = SeedableRng::seed_from_u64(0);

            let $arg: Vec<$typ> = (0usize .. LEN).map(|_| $gen(&mut rng)).collect();
            $(let $args: Vec<$types> = (0usize .. LEN).map(|_| $gens(&mut rng)).collect();)*
            let mut i = 0;

            bh.iter(|| {
                i = (i + 1) & (LEN - 1);

                unsafe {
                    let val: $tres = test::black_box($arg.get_unchecked(i).$method($(unref($args.get_unchecked(i)),)*));
                    drop(val);
                }
            })
        }
    }
);
