#![warn(non_camel_case_types)]

extern crate num;
extern crate rand;
extern crate time;
extern crate sfml;
extern crate mpeg_encoder;
extern crate nalgebra as na;
extern crate ncollide;


pub use testbed::Testbed;
pub use graphics_manager::{GraphicsManager, GraphicsManagerHandle};

mod testbed;
mod graphics_manager;

mod camera;
// mod fps;
mod draw_helper;

pub mod objects;
