extern crate rand;
extern crate num_traits as num;
extern crate glfw;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide;

#[cfg(feature = "recording")]
extern crate mpeg_encoder;


pub use testbed::Testbed;
pub use graphics_manager::{GraphicsManager, GraphicsManagerHandle};

mod testbed;
mod graphics_manager;
pub mod objects;
