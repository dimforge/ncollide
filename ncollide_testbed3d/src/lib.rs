extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate num_traits as num;
extern crate rand;
extern crate time;

#[cfg(feature = "recording")]
extern crate mpeg_encoder;

pub use graphics_manager::{GraphicsManager, GraphicsManagerHandle};
pub use testbed::Testbed;

mod graphics_manager;
pub mod objects;
mod testbed;
