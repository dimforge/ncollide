use sfml::system::{Vector2f, Vector2i};
use sfml::graphics::{Font, Text, Color, RenderTarget, Drawable, Transformable};
use sfml::graphics;
use std::time::{Duration, Instant};

pub struct Fps<'a> {
    delta:     Duration,
    last_time: Instant,
    fps:       Text<'a>
}

impl<'a> Fps<'a> {
    pub fn new(font: &'a Font) -> Fps<'a> {
        let mut txt = Text::new().unwrap();
        txt.set_font(font);
        txt.set_position(&Vector2f::new(0.0, 0.0));
        txt.set_color(&Color::new_rgb(255, 255, 255));

        Fps {
            delta:     Duration::new(0, 0),
            last_time: Instant::now(),
            fps:       txt
        }
    }

    pub fn reset(&mut self) {
        self.last_time = Instant::now();
    }

    pub fn register_delta(&mut self) {
        self.delta = self.last_time.elapsed()
    }

    pub fn elapsed_seconds(&self) -> f64 {
        let elapsed = self.last_time.elapsed();
        elapsed.as_secs() as f64 + elapsed.subsec_nanos() as f64 / 1000_000_000.0
    }

    pub fn draw_registered(&mut self, rw: &mut graphics::RenderWindow) {
        let elapsed = self.delta.as_secs() as f64 + self.delta.subsec_nanos() as f64 / 1000_000_000.0;

        let v = rw.get_view();

        self.fps.set_position(&rw.map_pixel_to_coords(&Vector2i { x: 0, y : 0 }, &v));
        self.fps.set_string(&format!("Time: {:.*}sec.", 4, elapsed)[..]);
        rw.draw(&self.fps);
    }
}
