use nalgebra::na;
use geom::mesh::MeshElement;
use math::{N, V};

#[deriving(Encodable, Decodable, Clone)]
pub struct Triangle {
    margin: N,
    a:      V,
    b:      V,
    c:      V
}

impl Triangle {
    #[inline]
    pub fn new(a: V, b: V, c: V) -> Triangle {
        Triangle::new_with_margin(a, b, c, na::cast(0.04))
    }

    #[inline]
    pub fn new_with_margin(a: V, b: V, c: V, margin: N) -> Triangle {
        assert!(na::dim::<V>() > 1);

        Triangle {
            margin: margin,
            a:      a,
            b:      b,
            c:      c
        }
    }
}

impl Triangle {
    #[inline]
    pub fn a<'a>(&'a self) -> &'a V {
        &'a self.a
    }

    #[inline]
    pub fn b<'a>(&'a self) -> &'a V {
        &'a self.b
    }

    #[inline]
    pub fn c<'a>(&'a self) -> &'a V {
        &'a self.c
    }

    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }
}

impl MeshElement for Triangle {
    #[inline]
    fn nvertices(_: Option<Triangle>) -> uint {
        3
    }

    #[inline]
    fn new_with_vertices_and_indices(vs: &[V], is: &[uint], margin: N) -> Triangle {
        assert!(is.len() == 3);

        Triangle::new_with_margin(vs[is[0]].clone(), vs[is[1]].clone(), vs[is[2]].clone(), margin)
    }
}
