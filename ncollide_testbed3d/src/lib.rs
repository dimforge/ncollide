extern crate rand;
extern crate time;
extern crate glfw;
extern crate mpeg_encoder;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide;


pub use testbed::Testbed;
pub use graphics_manager::{GraphicsManager, GraphicsManagerHandle};

mod testbed;
mod graphics_manager;
pub mod objects;
