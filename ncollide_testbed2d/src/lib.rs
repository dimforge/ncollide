#![warn(non_camel_case_types)]

extern crate num;
extern crate rand;
extern crate sfml;
extern crate nalgebra as na;
extern crate ncollide;

#[cfg(feature = "recording")]
extern crate mpeg_encoder;


pub use testbed::Testbed;
pub use graphics_manager::{GraphicsManager, GraphicsManagerHandle};

mod testbed;
mod graphics_manager;

mod camera;
// mod fps;
mod draw_helper;

pub mod objects;
