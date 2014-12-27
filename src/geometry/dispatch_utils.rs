#![macro_escape]

// FIXME: all this should be simplified when conditional dispatch is done.

macro_rules! apply_with_mixed_args(
    ($mname: ident,
     $arg0_plane_against_smap: ident $(, $args_plane_against_smap: ident)* |
     $arg0_smap_against_plane: ident $(, $args_smap_against_plane: ident)* |
     $arg0_smap_against_smap: ident $(, $args_smap_against_smap: ident)* |
     $arg0_concave_against_shape: ident $(, $args_concave_against_shape: ident)* |
     $arg0_shape_against_concave: ident $(, $args_shape_against_concave: ident)*
     ) => {
       /*
        * Plane against SupportMap.
        */
       $mname!($arg0_plane_against_smap $(, $args_plane_against_smap)* | Plane<V>, Ball<N>);
       $mname!($arg0_plane_against_smap $(, $args_plane_against_smap)* | Plane<V>, Cuboid<V>);
       $mname!($arg0_plane_against_smap $(, $args_plane_against_smap)* | Plane<V>, Capsule<N>);
       $mname!($arg0_plane_against_smap $(, $args_plane_against_smap)* | Plane<V>, Cone<N>);
       $mname!($arg0_plane_against_smap $(, $args_plane_against_smap)* | Plane<V>, Cylinder<N>);
       $mname!($arg0_plane_against_smap $(, $args_plane_against_smap)* | Plane<V>, Convex<P>);
       $mname!($arg0_plane_against_smap $(, $args_plane_against_smap)* | Plane<V>, Segment<P>);
       $mname!($arg0_plane_against_smap $(, $args_plane_against_smap)* | Plane<V>, Triangle<P>);

       /*
        * SupportMap against Plane.
        */
       $mname!($arg0_smap_against_plane $(, $args_smap_against_plane)* | Ball<N>, Plane<V>);
       $mname!($arg0_smap_against_plane $(, $args_smap_against_plane)* | Cuboid<V>, Plane<V>);
       $mname!($arg0_smap_against_plane $(, $args_smap_against_plane)* | Capsule<N>, Plane<V>);
       $mname!($arg0_smap_against_plane $(, $args_smap_against_plane)* | Cone<N>, Plane<V>);
       $mname!($arg0_smap_against_plane $(, $args_smap_against_plane)* | Cylinder<N>, Plane<V>);
       $mname!($arg0_smap_against_plane $(, $args_smap_against_plane)* | Convex<P>, Plane<V>);
       $mname!($arg0_smap_against_plane $(, $args_smap_against_plane)* | Segment<P>, Plane<V>);
       $mname!($arg0_smap_against_plane $(, $args_smap_against_plane)* | Triangle<P>, Plane<V>);

       /*
        * SupportMap against SupportMap.
        */
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Ball<N>, Cuboid<V>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Ball<N>, Capsule<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Ball<N>, Cone<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Ball<N>, Cylinder<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Ball<N>, Convex<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Ball<N>, Segment<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Ball<N>, Triangle<P>);

       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cuboid<V>, Ball<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cuboid<V>, Cuboid<V>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cuboid<V>, Capsule<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cuboid<V>, Cone<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cuboid<V>, Cylinder<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cuboid<V>, Convex<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cuboid<V>, Segment<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cuboid<V>, Triangle<P>);

       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Capsule<N>, Ball<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Capsule<N>, Cuboid<V>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Capsule<N>, Capsule<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Capsule<N>, Cone<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Capsule<N>, Cylinder<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Capsule<N>, Convex<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Capsule<N>, Segment<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Capsule<N>, Triangle<P>);

       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cone<N>, Ball<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cone<N>, Cuboid<V>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cone<N>, Capsule<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cone<N>, Cone<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cone<N>, Cylinder<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cone<N>, Convex<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cone<N>, Segment<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cone<N>, Triangle<P>);

       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cylinder<N>, Ball<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cylinder<N>, Cuboid<V>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cylinder<N>, Capsule<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cylinder<N>, Cone<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cylinder<N>, Cylinder<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cylinder<N>, Convex<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cylinder<N>, Segment<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Cylinder<N>, Triangle<P>);

       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Convex<P>, Ball<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Convex<P>, Cuboid<V>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Convex<P>, Capsule<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Convex<P>, Cone<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Convex<P>, Cylinder<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Convex<P>, Convex<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Convex<P>, Segment<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Convex<P>, Triangle<P>);

       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Segment<P>, Ball<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Segment<P>, Cuboid<V>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Segment<P>, Capsule<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Segment<P>, Cone<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Segment<P>, Cylinder<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Segment<P>, Convex<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Segment<P>, Segment<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Segment<P>, Triangle<P>);

       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Triangle<P>, Ball<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Triangle<P>, Cuboid<V>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Triangle<P>, Capsule<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Triangle<P>, Cone<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Triangle<P>, Cylinder<N>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Triangle<P>, Convex<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Triangle<P>, Segment<P>);
       $mname!($arg0_smap_against_smap $(, $args_smap_against_smap)* | Triangle<P>, Triangle<P>);

       /*
        * Concave against Shape.
        */
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Triangle<P>>, Ball<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Triangle<P>>, Cuboid<V>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Triangle<P>>, Capsule<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Triangle<P>>, Cone<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Triangle<P>>, Cylinder<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Triangle<P>>, Convex<P>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Triangle<P>>, Segment<P>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Triangle<P>>, Triangle<P>);

       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Segment<P>>, Ball<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Segment<P>>, Cuboid<V>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Segment<P>>, Capsule<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Segment<P>>, Cone<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Segment<P>>, Cylinder<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Segment<P>>, Convex<P>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Segment<P>>, Segment<P>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Mesh<N, P, V, Segment<P>>, Triangle<P>);

       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Ball<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Cuboid<V>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Capsule<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Cone<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Cylinder<N>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Convex<P>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Segment<P>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Triangle<P>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Plane<V>);

       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Compound<N, P, V, M>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Mesh<N, P, V, Segment<P>>);
       $mname!($arg0_concave_against_shape $(, $args_concave_against_shape)* | Compound<N, P, V, M>, Mesh<N, P, V, Triangle<P>>);

       /*
        * Shape against Concave.
        */
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Ball<N>, Mesh<N, P, V, Triangle<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cuboid<V>, Mesh<N, P, V, Triangle<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Capsule<N>, Mesh<N, P, V, Triangle<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cone<N>, Mesh<N, P, V, Triangle<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cylinder<N>, Mesh<N, P, V, Triangle<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Convex<P>, Mesh<N, P, V, Triangle<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Segment<P>, Mesh<N, P, V, Triangle<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Triangle<P>, Mesh<N, P, V, Triangle<P>>);

       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Ball<N>, Mesh<N, P, V, Segment<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cuboid<V>, Mesh<N, P, V, Segment<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Capsule<N>, Mesh<N, P, V, Segment<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cone<N>, Mesh<N, P, V, Segment<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cylinder<N>, Mesh<N, P, V, Segment<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Convex<P>, Mesh<N, P, V, Segment<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Segment<P>, Mesh<N, P, V, Segment<P>>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Triangle<P>, Mesh<N, P, V, Segment<P>>);

       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Ball<N>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cuboid<V>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Capsule<N>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cone<N>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Cylinder<N>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Convex<P>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Segment<P>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Triangle<P>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Plane<V>, Compound<N, P, V, M>);

       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Mesh<N, P, V, Segment<P>>, Compound<N, P, V, M>);
       $mname!($arg0_shape_against_concave $(, $args_shape_against_concave)* | Mesh<N, P, V, Triangle<P>>, Compound<N, P, V, M>);
       // $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Compound<N, P, V, M>)
    }
);

macro_rules! apply_to_all_shape_pair(
    ($mname: ident, $arg0: ident $(, $args: ident)*) => {
        {
            /*
             * Ball against Ball.
             */
            $mname!($arg0 $(, $args)* | Ball<N>, Ball<N>);

            /*
             * Plane against SupportMap.
             */
            $mname!($arg0 $(, $args)* | Plane<V>, Ball<N>);
            $mname!($arg0 $(, $args)* | Plane<V>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Plane<V>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Plane<V>, Cone<N>);
            $mname!($arg0 $(, $args)* | Plane<V>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Plane<V>, Convex<P>);
            $mname!($arg0 $(, $args)* | Plane<V>, Segment<P>);
            $mname!($arg0 $(, $args)* | Plane<V>, Triangle<P>);

            /*
             * SupportMap against Plane.
             */
            $mname!($arg0 $(, $args)* | Ball<N>, Plane<V>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Plane<V>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Plane<V>);
            $mname!($arg0 $(, $args)* | Cone<N>, Plane<V>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Plane<V>);
            $mname!($arg0 $(, $args)* | Convex<P>, Plane<V>);
            $mname!($arg0 $(, $args)* | Segment<P>, Plane<V>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Plane<V>);

            /*
             * SupportMap against SupportMap.
             */
            $mname!($arg0 $(, $args)* | Ball<N>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Ball<N>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Ball<N>, Cone<N>);
            $mname!($arg0 $(, $args)* | Ball<N>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Ball<N>, Convex<P>);
            $mname!($arg0 $(, $args)* | Ball<N>, Segment<P>);
            $mname!($arg0 $(, $args)* | Ball<N>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Cuboid<V>, Ball<N>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Cone<N>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Convex<P>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Segment<P>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Capsule<N>, Ball<N>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Cone<N>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Convex<P>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Segment<P>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Cone<N>, Ball<N>);
            $mname!($arg0 $(, $args)* | Cone<N>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Cone<N>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Cone<N>, Cone<N>);
            $mname!($arg0 $(, $args)* | Cone<N>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Cone<N>, Convex<P>);
            $mname!($arg0 $(, $args)* | Cone<N>, Segment<P>);
            $mname!($arg0 $(, $args)* | Cone<N>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Cylinder<N>, Ball<N>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Cone<N>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Convex<P>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Segment<P>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Convex<P>, Ball<N>);
            $mname!($arg0 $(, $args)* | Convex<P>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Convex<P>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Convex<P>, Cone<N>);
            $mname!($arg0 $(, $args)* | Convex<P>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Convex<P>, Convex<P>);
            $mname!($arg0 $(, $args)* | Convex<P>, Segment<P>);
            $mname!($arg0 $(, $args)* | Convex<P>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Segment<P>, Ball<N>);
            $mname!($arg0 $(, $args)* | Segment<P>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Segment<P>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Segment<P>, Cone<N>);
            $mname!($arg0 $(, $args)* | Segment<P>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Segment<P>, Convex<P>);
            $mname!($arg0 $(, $args)* | Segment<P>, Segment<P>);
            $mname!($arg0 $(, $args)* | Segment<P>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Triangle<P>, Ball<N>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Cone<N>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Convex<P>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Segment<P>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Triangle<P>);

            /*
             * Concave against Shape.
             */
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Ball<N>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Cone<N>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Convex<P>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Segment<P>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Ball<N>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Cone<N>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Convex<P>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Segment<P>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Triangle<P>);

            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Ball<N>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Cuboid<V>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Capsule<N>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Cone<N>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Cylinder<N>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Convex<P>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Segment<P>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Triangle<P>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Plane<V>);

            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Mesh<N, P, V, Segment<P>>);
            $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Mesh<N, P, V, Triangle<P>>);

            /*
             * Shape against Concave.
             */
            $mname!($arg0 $(, $args)* | Ball<N>, Mesh<N, P, V, Triangle<P>>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Mesh<N, P, V, Triangle<P>>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Mesh<N, P, V, Triangle<P>>);
            $mname!($arg0 $(, $args)* | Cone<N>, Mesh<N, P, V, Triangle<P>>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Mesh<N, P, V, Triangle<P>>);
            $mname!($arg0 $(, $args)* | Convex<P>, Mesh<N, P, V, Triangle<P>>);
            $mname!($arg0 $(, $args)* | Segment<P>, Mesh<N, P, V, Triangle<P>>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Mesh<N, P, V, Triangle<P>>);

            $mname!($arg0 $(, $args)* | Ball<N>, Mesh<N, P, V, Segment<P>>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Mesh<N, P, V, Segment<P>>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Mesh<N, P, V, Segment<P>>);
            $mname!($arg0 $(, $args)* | Cone<N>, Mesh<N, P, V, Segment<P>>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Mesh<N, P, V, Segment<P>>);
            $mname!($arg0 $(, $args)* | Convex<P>, Mesh<N, P, V, Segment<P>>);
            $mname!($arg0 $(, $args)* | Segment<P>, Mesh<N, P, V, Segment<P>>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Mesh<N, P, V, Segment<P>>);

            $mname!($arg0 $(, $args)* | Ball<N>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Cuboid<V>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Capsule<N>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Cone<N>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Cylinder<N>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Convex<P>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Segment<P>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Triangle<P>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Plane<V>, Compound<N, P, V, M>);

            // $mname!($arg0 $(, $args)* | Compound<N, P, V, M>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Segment<P>>, Compound<N, P, V, M>);
            $mname!($arg0 $(, $args)* | Mesh<N, P, V, Triangle<P>>, Compound<N, P, V, M>);
        }
    }
);

