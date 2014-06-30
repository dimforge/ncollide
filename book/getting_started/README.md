# Getting started

## Compilation
1. Download and install the latest nightly distribution of the [rust
   compiler](http://rust-lang.org). **ncollide** is regularly kept up to date with
   those releases. If it is not, this is a bug that you can file on
   [github](https://github.com/sebcrozet/ncollide/issues).
2. Recursively clone the repository of ncollide:
```sh
git clone --recursive git://github.com/sebcrozet/ncollide.git
```
Performing a recursive clone as such will automatically clone the rust library
[nalgebra](http://nalgebra.org) that **ncollide** uses for linear algebra.
3. Go to the root of the repository and compile the dependencies:
```sh
cd ncollide
make deps
```
4. Finally compile the library itself:
```
make
```
This will output six libraries to the `lib` subdirectory:
    * **ncollide2df32** for 2d collision detection using single-precision numbers.
    * **ncollide3df32** for 3d collision detection using single-precision numbers.
    * **ncollide4df32** for 4d collision detection using single-precision numbers.
    * **ncollide2df64** for 2d collision detection using double-precision numbers.
    * **ncollide3df64** for 3d collision detection using double-precision numbers.
    * **ncollide4df64** for 4d collision detection using double-precision numbers.

If you are not interested in all the variants of **ncollide**, you can
selectively compile one of them using `make <dimension>df<precision>`. For
example, to compile the 3d version using 32-bits floating point numbers, use
`make 3df32`.

The full documentation of the api can be built manually using rustdoc or with
`make doc`.

## Using ncollide on your project
You can use **ncollide** like any other rust library. You first have to import
the crate that you are interested in:
```rust
extern crate ncollide3df32
```
Then, you have to compile your project, making sure that the imported crate is
in the library search path using rustc's `-L $(your_path_to_ncollide)` option.

Having to write the dimension and the precision used by the library every time
you import one of its functionality might be very cumbersome: `use
ncollide3df32::geom::Cuboid;`. A nicer route is to use an alias when the crate
is imported:
```rust
extern crate ncollide = "ncollide3df32"
```
This way, you only have to write `use ncollide::geom::Cuboid;`.

###### Working example <button style="float:right;" class="btn btn-primary" type="button" id="download-code" onclick="window.open('../src/getting_started.rs')"><img style="float:left;width:20px;height:20px;" src="../img/d.svg" /></button>
```rust
extern crate nalgebra;
extern crate ncollide = "ncollide3df32";

use nalgebra::na::Vec3;
use ncollide::geom::Cuboid;
use ncollide::ray::{Ray, RayCast};

fn main() {
    let cube = Cuboid::new(Vec3::new(1.0f32, 1.0, 1.0));
    let ray  = Ray::new(Vec3::new(0.0f32, 0.0, -1.0), Vec3::z());

    assert!(cube.intersects_ray(&ray));
}
```
