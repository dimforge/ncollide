# How to contribute

### Technical contribution

You are of course welcome to contribute to the source code of **ncollide** and to
this guide. Simply make sure to [open an
issue](https://github.com/sebcrozet/ncollide/issues) on GitHub if you intend to
perform a large contribution. This should prevent other people from stepping
silently on your toes and should ensure your work is going to be merged when it
is ready.

###### Working on the rust project

If you intend to work on the source code of **ncollide**, you should start by
[forking](https://help.github.com/articles/fork-a-repo) the
[repository](https://github.com/sebcrozet/ncollide). All the main developments
are done on the `master` branch.  Once you are done making modifications to
your own copy of **ncollide**, you have to [create a
pull⁻request](https://help.github.com/articles/creating-a-pull-request) so that
your contribution can be reviewed, commented, and merged.


If you read this guide completely, you may have noticed that the first rule of
development is to expose each feature by a trait located on its own module.
Thus, you will usually find one folder per major feature. Each folder contains
a similarly named file which declares the relevant traits. Finally, those
traits are implemented for the relevant structures on explicitly named files
located on the very same folder.  For example, the `Implicit` trait is declared
on the `implicit/implicit.rs` file and its implementation for e.g.  the `Ball`
geometry is located on `implicit/implicit_ball.rs` while its implementation for
e.g. the `Cuboid` geometry is located on `implicit/implicit_cuboid.rs`. You
will find similar naming patterns for most features.


Another rule is to never use explicit scalar types like `f32` and `f64`.
Explicit vector and matrix types should be avoided too when possible. Instead,
use the aliases defined by the `math` module. They automatically change
depending on the version of **ncollide** chosen by the user:

| Alias name | Value for 2df32 | Value for 2df64 | Value for 3df32 | Value for 3df64 |
| --         | --              | --              | --              | --              |
| `Scalar`         | `f32`           | `f64`           | `f32`           | `f64` |
| `Vect`           | `Vec2<f32>`     | `Vec2<f64>`     | `Vec3<f32>`     | `Vec3<f64>` |
| `Orientation`    | `Vec1<f32>`     | `Vec1<f64>`     | `Vec3<f32>`     | `Vec3<f64>` |
| `Matrix`         | `Iso2<f32>`     | `Iso2<f64>`     | `Iso3<f32>`     | `Iso3<f64>` |
| `RotationMatrix` | `Rot2<f32>`     | `Rot2<f64>`     | `Rot3<f32>`     | `Rot3<f64>` |
| `AngularInertia` | `Mat1<f32>`     | `Mat1<f64>`     | `Mat3<f32>`     | `Mat3<f64>` |

If you need to convert a constant like `42.0` to a `math::Scalar`, use the
function `na::cast` from nalgebra:

```rust
let value: Scalar = na::cast(42.0);
```


Finally, if you need to implement something that is dimension-specific (i.e.
2D-only or 3D-only), use one (and only one) of the following attributes with
your `fn`, `impl`, `struct` and `trait` definitions:

| Attribute | Effect |
| --        | --     |
| `#[dim2]` | Compiles the affected code only for the 2D version of **ncollide**. |
| `#[dim3]` | Compiles the affected code only for the 3D version of **ncollide**. |
| `#[dim4]` | Compiles the affected code only for the 4D version of **ncollide**. |
| `#[not_dim2]` | Compiles the affected code only for all but the 2D version of **ncollide**. |
| `#[not_dim3]` | Compiles the affected code only for all but the 3D version of **ncollide**. |
| `#[not_dim4]` | Compiles the affected code only for all but the 4D version of **ncollide**. |

Here is an example of dummy code using those attributes:

```rust
use nalgebra::na;
use math::Vect; // no `ncollide::`, we assume file is on the source tree of ncollide.

#[dim2]
fn doit() { // compile this implementation for the 2D case.
    assert!(na::dim::<Vect>() == 2);
}

#[dim3]
fn doit() { // compile this implementation for the 3D case.
    assert!(na::dim::<Vect>() == 3);
}

#[dim4]
fn doit() { // compile this implementation for the 4D case.
    assert!(na::dim::<Vect>() == 4);
}
```

Note that those attributes will not work on `use` statements. Thus, you might
end up with _legitimate_ unused import warnings (we will not blame you for
letting them).


###### Working on this guide

You can also contribute to this user guide by completing and correcting it. Do
not hesitate to correct even the smallest, insignificant detail (especially
English mistakes). We love nitpicking!
This guide is composed of a set of markdown files located on the
[gh-pages](https://github.com/sebcrozet/ncollide/tree/gh-pages) branch of the
main **ncollide** repository. It is compiled manually using
[GitBook](https://www.gitbook.io/). As explained before, you need to fork, fix,
and create a pull request to make your contribution upstreamable. There are no
specific rules, except that all compilable and downloadable examples must be
located on the `src` folder.


### Financial contribution

Donations made to the lead developer (more informations about him bellow)
are also appreciated. However, do not forget that **donating is not a
requirement**. You are, and will always be free to use **ncollide** for any
purpose, including commercial applications, without paying anything (see the
[BSD-3 licence](https://github.com/sebcrozet/ncollide/blob/master/LICENSE)).
<div style="float:right">
<table style="border-style:none" align="center">
<tr style="border-style:none">
<td style="border-style:none">
<a href="https://flattr.com/submit/auto?user_id=sebcrozet&url=http%3A%2F%2Fncollide.org" target="_blank"><img src="https://api.flattr.com/button/flattr-badge-large.png" alt="Flattr this" title="Flattr this" border="0"></a>
</td>
<td style="border-style:none">
<center>
<form action="https://www.paypal.com/cgi-bin/webscr" method="post" target="_top">
<input type="hidden" name="cmd" value="_s-xclick">
<input type="hidden" name="encrypted" value="-----BEGIN PKCS7-----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-----END PKCS7-----
">
<input type="image" src="https://www.paypalobjects.com/en_US/i/btn/btn_donate_SM.gif" border="0" name="submit" alt="PayPal - The safer, easier way to pay online!">
</form>
</tr>
</center>
</td>
</table>
</div>

# Contributors

As of today, this reference guide and the **ncollide** project are almost
completely developed by me, a French Computer Science student named Sébastien
Crozet (aka. _sebcrozet_). I do that during my free time, trying to help making
Rust a great language for the development of real-time geometry-intensive
applications. For more informations about my professional background, check out
my [LinkedIn](https://www.linkedin.com/pub/s%C3%A9bastien-crozet/25/875/416/en)
profile.


I also develop a physics engine [nphysics](http://nphysics-dev.org), a
graphics engine [kiss3d](http://kiss3d.org), and a low-dimensional linear
algebra library [nalgebra](http://nalgebra.org). Do not hesitate to use those
as freely as you use **ncollide**!

-----

Except me, here are the known contributors (including donors) to the
**ncollide** project:

* [Alexander Mistakidis](https://github.com/aamistak)
* [Ben Foppa](https://github.com/bfops)
* [Brandon Waskiewicz](https://github.com/brandonw)
* [Peter Nguyen](https://github.com/cacteye)

If your name should or should not be on this list, or if it should link to
another website, please send me an [email](mailto:developer@crozet.re) or open
an issue to let me know.

