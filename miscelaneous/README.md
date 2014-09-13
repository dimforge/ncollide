# Miscelanous

Besides the main collision detection and mesh generation related features,
**ncollide** exposes a number of unclassified geometric operations that are
used internally by the library. Those operations are exported by the `utils`
module. Here are listed the most useful operations, most of them being
dimension-generic (no matter which version of **ncollide** you use):

| Function                    | Description                                          |
|--                           | --                                                   |
| `center(pts)`               | Computes the center of the points `pts`.             |
| `circumcircle(a, b, c)`     | Computes the circumcircle of the triangle `a, b, c`. |
| `cov(pts)`                  | Computes the covariance matrix of the points `pts`.  |
| `is_point_in_triangle(...)` | Tests that a point is inside of a triangle.          |
| `sort3(a, b, c)`            | Sorts in increasing order a set of three values.     |
| `triangulate(pts)`          | Triangulates the points `pts`.                       |

Some optimization-related functions are also implemented:

| Function      | Description                                          |
|--             | --                                                   |
| `newton(...)` | Finds a root of a function using the [Newton](http://en.wikipedia.org/wiki/Newton's_method) method.  |
| `bfgs(...)`   | Minimizes a function using the [BFGS](http://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm) method. |

See the
[documentation](http://ncollide.org/doc/ncollide3df32/utils/index.html) for an
exhaustive list.


### The differentiation DSL

The `utils::symbolic` module defines a very inefficient but useful
differentiation Domain Specific Language. In other words, it is a set of
structures and function that allows you to describe symbolically a one or
two-variables real function and evaluate their derivative at any order.

For example, if you want to compute the tenth derivative at the point $$t =
0$$ of the function $$f(t) = 10t + sin(cos(t + 3)t)$$, you simply have to
import the mathematical functions and the `UnivariateFn` trait:

```rust
use ncollide::utils::symbolic::{UnivariateFn, sin, cos, t};
```

Define your function symbolically:

```rust
let t = t();
let f = t * 10.0 + sin(cos(t + 3.0) * t);
```

And evaluate its tenth derivative for $$t = 0.0$$:

```rust
f.dn(0.0f32, 10);
```

The `UnivariateFn` trait allows you to call `.dn(...)` and other methods
optimized for low-order derivatives:

| Method               | Description |
|--                    | --          |
| `.d0(t)`             | Evaluates the function at the point `t`. |
| `.d1(t)`             | Evaluates the first derivative at the point `t`.  |
| `.d2(t)`             | Evaluates the second derivative at the point `t`. |
| `.dn(t, n)`          | Evaluates the `n`th derivative at the point `t`. |
| `.ueval(t)`          | Same as `.d0(t)`. Use this to avoid clashes when the `UnivariateFn` and the `BivariateFn` traits are imported simultaneously. |
| `.d0_1(t)`           | Evaluates simultaneously the function and its first derivative at the point `t`. |
| `.d0_1_2(t)`         | Evaluates simultaneously the function and its first two derivatives at the point `t`. |
| `.dn_all(t, n, out)` | Evaluates simultaneously the function and all its first `n`th derivatives at the point `t`. The values are output to the vector `out`. |

It is possible to achieve similar results with two-variable real functions and
compute their $$n$$th partial derivative with regard to the first variable `u`,
and their $$k$$th partial derivative with regard to the second variable `v`:

```rust
let u = u();
let v = v();
let f = u + sin(u) * cos(u - u * (v + u) * 2.0);
f.duv_nk(1.0f32, 2.0, 10, 8);
```

To use the method `.duv_nk(...)`, the `BivariateFn` must be imported:


| Method                        | Description |
|--                             | --          |
| `.d0(u, v)`                    | Evaluates the function at the point `u, v`. |
| `.du(u, v)`                    | Evaluates the first partial derivative wrt. `u` at the point `u, v`. |
| `.dv(u, v)`                    | Evaluates the first partial derivative wrt. `v` at the point `u, v`. |
| `.duu(u, v)`                   | Evaluates the second partial derivative wrt. `u` at the point `u, v`. |
| `.dvv(u, v)`                   | Evaluates the second partial derivative wrt. `v` at the point `u, v`. |
| `.duv(u, v)`                   | Evaluates the first partial derivative wrt. `u` and `v` at the point `u, v`. |
| `.duv_nk(u, v, n, k)`          | Evaluates the `n`th (resp. `k`th) partial derivative wrt. `u` (resp. `v`) at the point `u, v`. |
| `.beval(u, v)`                 | Same as `.d0(u, v)`. Use this to avoid clashes when the `UnivariateFn` and the `BivariateFn` traits are imported simultaneously. |
| `.duv_nk_all(u, v, n, k, out)` | Computes all the `n`th (resp. `k`th) partial derivatives wrt. `u` (resp. `v`) at the point `u, v`. The result is output to the matrix `out`. Its $$i$$th (resp. $$j$$th) row (resp. column) corresponds to the $$i$$th (resp. $$j$$th) partial derivative wrt. `u` (resp. `v`).|

Note that this implementation is very naive and does performs any kind of
simplification or memoization. In the future it might be reimplemented using
macros.

###### Example <span class="btn-primary" onclick="window.open('../src/dsl.rs')"></span>

```rust
let u = u();
let v = v();
let t = t();

let f1 = t * t + t * 2.0f32;
let f2 = u * v + u * 2.0f32;

assert!(f1.ueval(1.0f32) == 3.0);
assert!(f1.d1(1.0f32)    == 4.0);
assert!(f1.d2(1.0f32)    == 2.0);
assert!(f1.dn(1.0f32, 3) == 0.0);

assert!(f2.beval(1.0f32, 2.0)        == 4.0);
assert!(f2.du(1.0f32, 2.0)           == 4.0);
assert!(f2.dv(1.0f32, 2.0)           == 1.0);
assert!(f2.duu(1.0f32, 2.0)          == 0.0);
assert!(f2.dvv(1.0f32, 2.0)          == 0.0);
assert!(f2.duv(1.0f32, 2.0)          == 1.0);
assert!(f2.duv_nk(1.0f32, 2.0, 3, 0) == 0.0);
```
