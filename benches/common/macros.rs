#![macro_escape]

macro_rules! bench_free_fn(
    ($name: ident, $function: path $(, $args: ident: $types: ty)*) => {
        bench_free_fn_gen!($name, $function $(, $args: $types = random)*)
    }
)

macro_rules! bench_method(
    ($name: ident, $method: ident, $arg: ident: $typ: ty $(, $args: ident: $types: ty)*) => {
        bench_method_gen!($name, $method, $arg: $typ = random $(, $args: $types = random)*)
    }
)

macro_rules! bench_free_fn_gen(
    ($name: ident, $function: path $(, $args: ident: $types: ty = $gens: path)*) => {
        #[bench]
        fn $name(bh: &mut Bencher) {
            const LEN: uint = 1 << 7;

            let mut rng = IsaacRng::new_unseeded();

            $(let $args: Vec<$types> = Vec::from_fn(LEN, |_| $gens(&mut rng));)*
            let mut i = 0;

            bh.iter(|| {
                i = (i + 1) & (LEN - 1);

                unsafe {
                    test::black_box($function($(unref($args.unsafe_get(i)),)*))
                }
            });
        }
    }
)

macro_rules! bench_method_gen(
    ($name: ident, $method: ident, $arg: ident: $typ: ty = $gen: path $(, $args: ident: $types: ty = $gens: path)*) => {
        #[bench]
        fn $name(bh: &mut Bencher) {
            const LEN: uint = 1 << 7;

            let mut rng = IsaacRng::new_unseeded();

            let $arg: Vec<$typ> = Vec::from_fn(LEN, |_| $gen(&mut rng));
            $(let $args: Vec<$types> = Vec::from_fn(LEN, |_| $gens(&mut rng));)*
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
