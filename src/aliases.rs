pub mod dim4 {
    use bounding_volume::AABB;
    use geom::{Ball, Box, Cylinder, Cone, Plane, Compound, Triangle, Mesh, Geom, Segment};
    use nalgebra::na::{Vec4, Mat4, Iso4};

    type LV<N> = Vec4<N>;
    type II<N> = Mat4<N>;
    type M<N>  = Iso4<N>;

    pub type Ball4d<N>         = Ball<N>;
    pub type Box4d<N>          = Box<N, LV<N>>;
    pub type Cylinder4d<N>     = Cylinder<N>;
    pub type Cone4d<N>         = Cone<N>;
    pub type Plane4d<N>        = Plane<N, LV<N>>;
    pub type Compound4d<N>     = Compound<N, LV<N>, M<N>, II<N>>;
    pub type Triangle4d<N>     = Triangle<N, LV<N>>;
    pub type Segment4d<N>      = Segment<N, LV<N>>;
    pub type TriangleMesh4d<N> = Mesh<N, LV<N>, M<N>, II<N>, Triangle4d<N>>;
    pub type LineStrip4d<N>    = Mesh<N, LV<N>, M<N>, II<N>, Segment4d<N>>;
    pub type Geom4dRef<'a, N>  = &'a Geom<N, LV<N>, M<N>, II<N>>;
    pub type Geom4d<N>         = ~Geom<N, LV<N>, M<N>, II<N>>;
    pub type AABB4d<N>         = AABB<N, LV<N>>;
}

pub mod dim3 {
    use bounding_volume::AABB;
    use geom::{Ball, Box, Cylinder, Cone, Plane, Compound, Triangle, Mesh, Geom, Segment};
    use nalgebra::na::{Vec3, Mat3, Iso3};

    type LV<N> = Vec3<N>;
    type II<N> = Mat3<N>;
    type M<N>  = Iso3<N>;

    pub type Ball3d<N>         = Ball<N>;
    pub type Box3d<N>          = Box<N, LV<N>>;
    pub type Cylinder3d<N>     = Cylinder<N>;
    pub type Cone3d<N>         = Cone<N>;
    pub type Plane3d<N>        = Plane<N, LV<N>>;
    pub type Compound3d<N>     = Compound<N, LV<N>, M<N>, II<N>>;
    pub type Triangle3d<N>     = Triangle<N, LV<N>>;
    pub type Segment3d<N>      = Segment<N, LV<N>>;
    pub type TriangleMesh3d<N> = Mesh<N, LV<N>, M<N>, II<N>, Triangle3d<N>>;
    pub type LineStrip3d<N>    = Mesh<N, LV<N>, M<N>, II<N>, Segment3d<N>>;
    pub type Geom3dRef<'a, N>  = &'a Geom<N, LV<N>, M<N>, II<N>>;
    pub type Geom3d<N>         = ~Geom<N, LV<N>, M<N>, II<N>>;
    pub type AABB3d<N>         = AABB<N, LV<N>>;
}

pub mod dim2 {
    use bounding_volume::AABB;
    use geom::{Ball, Box, Cylinder, Cone, Plane, Compound, Triangle, Mesh, Geom, Segment};
    use nalgebra::na::{Mat1, Vec2, Iso2};

    type LV<N> = Vec2<N>;
    type II<N> = Mat1<N>;
    type M<N>  = Iso2<N>;

    pub type Ball2d<N>         = Ball<N>;
    pub type Box2d<N>          = Box<N, LV<N>>;
    pub type Cylinder2d<N>     = Cylinder<N>;
    pub type Cone2d<N>         = Cone<N>;
    pub type Plane2d<N>        = Plane<N, LV<N>>;
    pub type Compound2d<N>     = Compound<N, LV<N>, M<N>, II<N>>;
    pub type Triangle2d<N>     = Triangle<N, LV<N>>;
    pub type Segment2d<N>      = Segment<N, LV<N>>;
    pub type TriangleMesh2d<N> = Mesh<N, LV<N>, M<N>, II<N>, Triangle2d<N>>;
    pub type LineStrip2d<N>    = Mesh<N, LV<N>, M<N>, II<N>, Segment2d<N>>;
    pub type Geom2dRef<'a, N>  = &'a Geom<N, LV<N>, M<N>, II<N>>;
    pub type Geom2d<N>         = ~Geom<N, LV<N>, M<N>, II<N>>;
    pub type AABB2d<N>         = AABB<N, LV<N>>;
}
