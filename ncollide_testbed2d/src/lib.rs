#![warn(non_camel_case_types)]

extern crate alga;
extern crate nalgebra as na;
extern crate ncollide2d;
extern crate num_traits as num;
extern crate rand;
extern crate sfml;
extern crate time;

#[cfg(feature = "recording")]
extern crate mpeg_encoder;

pub use graphics_manager::{GraphicsManager, GraphicsManagerHandle};
pub use testbed::Testbed;

mod graphics_manager;
mod testbed;

mod camera;
// mod fps;
mod draw_helper;

pub mod objects;
