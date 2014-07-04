//!
//! Geometry composed from the union of primitives.
//!

use std::sync::Arc;
use nalgebra::na;
use bounding_volume::{LooseBoundingVolume, AABB, HasAABB};
use ray::Ray;
use volumetric::Volumetric;
use partitioning::BVT;
use partitioning::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};
use geom::{Geom, ConcaveGeom};
use math::{Scalar, Vect, Matrix, AngularInertia};

#[deriving(Clone)]
/// Structure used to build a `Compound` geometry.
///
/// This accumulates the geometries and their volumetric properties.
pub struct CompoundData {
    geoms: Vec<(Matrix, Arc<Box<Geom + Send + Share>>)>,
    props: Vec<(Scalar, Vect, AngularInertia)>
}

impl CompoundData {
    /// Creates a new `CompoundData`.
    pub fn new() -> CompoundData {
        CompoundData {
            geoms: Vec::new(),
            props: Vec::new()
        }
    }

    /// Adds a new geometry; its mass properties are automatically computed.
    #[inline]
    pub fn push_geom<S: Geom + Volumetric + Send + Share>(&mut self, delta: Matrix, geom: S, density: Scalar) {
        let props = geom.mass_properties(&density);

        self.push_geom_with_mass_properties(delta, geom, props)
    }

    /// Adds a new geometry with the given mass properties.
    #[inline]
    pub fn push_geom_with_mass_properties<S: Geom + Send + Share>(&mut self,
                                                                  delta: Matrix,
                                                                  geom:  S,
                                                                  props: (Scalar, Vect, AngularInertia)) {
        self.push_shared_geom_with_mass_properties(delta, Arc::new(box geom as Box<Geom + Send + Share>), props)
    }

    /// Adds a new shared geometry with the given mass properties.
    #[inline]
    pub fn push_shared_geom_with_mass_properties(&mut self,
                                                 delta: Matrix,
                                                 geom:  Arc<Box<Geom + Send + Share>>,
                                                 props: (Scalar, Vect, AngularInertia)) {
        self.geoms.push((delta, geom));
        self.props.push(props);
    }

    /// The geometries stored by this `CompoundData`.
    #[inline]
    pub fn geoms<'r>(&'r self) -> &'r [(Matrix, Arc<Box<Geom + Send + Share>>)] {
        self.geoms.as_slice()
    }

    // FIXME: this is not a very good name.
    /// The geometries stored by this `CompoundData`.
    #[inline]
    pub fn mass_properties_list<'r>(&'r self) -> &'r [(Scalar, Vect, AngularInertia)] {
        self.props.as_slice()
    }
}

/// A compound geometry with an aabb bounding volume.
///
/// A compound geometry is a geometry composed of the union of several simpler geometry. This is
/// the main way of creating a concave geometry from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other geometries.
#[deriving(Clone)]
pub struct Compound {
    mass:    Scalar,
    inertia: AngularInertia,
    com:     Vect,
    geoms:   Vec<(Matrix, Arc<Box<Geom + Send + Share>>)>,
    bvt:     BVT<uint, AABB>,
    bvs:     Vec<AABB>
}

impl Compound {
    /// Builds a new compound geometry.
    pub fn new(data: CompoundData) -> Compound {
        let mut bvs    = Vec::new();
        let mut leaves = Vec::new();

        for (i, &(ref delta, ref geom)) in data.geoms().iter().enumerate() {
            let bv = geom.aabb(delta).loosened(na::cast(0.04f64)); // loosen for better persistancy

            bvs.push(bv.clone());
            leaves.push((i, bv));
        }

        let bvt = BVT::new_kdtree(leaves);

        let (mass, com, inertia) = data.mass_properties(&na::one());

        Compound {
            mass:    mass,
            inertia: inertia,
            com:     com,
            geoms:   data.geoms,
            bvt:     bvt,
            bvs:     bvs
        }
    }
}

impl Compound {
    /// The geometries of this compound geometry.
    #[inline]
    pub fn geoms<'r>(&'r self) -> &'r [(Matrix, Arc<Box<Geom + Send + Share>>)] {
        self.geoms.as_slice()
    }

    /// The optimization structure used by this compound geometry.
    #[inline]
    pub fn bvt<'r>(&'r self) -> &'r BVT<uint, AABB> {
        &'r self.bvt
    }

    /// The geometries bounding volumes.
    #[inline]
    pub fn bounding_volumes<'r>(&'r self) -> &'r [AABB] {
        self.bvs.as_slice()
    }

    #[doc(hidden)]
    #[inline]
    pub fn mass(&self) -> Scalar {
        self.mass.clone()
    }

    #[doc(hidden)]
    #[inline]
    pub fn angular_inertia<'r>(&'r self) -> &'r AngularInertia {
        &'r self.inertia
    }

    #[doc(hidden)]
    #[inline]
    pub fn center_of_mass<'r>(&'r self) -> &'r Vect {
        &'r self.com
    }
}

impl ConcaveGeom for Compound {
    #[inline(always)]
    fn map_part_at<T>(&self, i: uint, f: |&Matrix, &Geom| -> T) -> T{
        let &(ref m, ref g) = self.geoms.get(i);

        f(m, **g)
    }

    #[inline(always)]
    fn map_transformed_part_at<T>(&self, m: &Matrix, i: uint, f: |&Matrix, &Geom| -> T) -> T{
        let &(ref lm, ref g) = self.geoms.get(i);

        f(&(m * *lm), **g)
    }

    #[inline]
    fn approx_interferences_with_aabb(&self, aabb: &AABB, out: &mut Vec<uint>) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(aabb, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn approx_interferences_with_ray(&self, ray: &Ray, out: &mut Vec<uint>) {
        let mut visitor = RayInterferencesCollector::new(ray, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn aabb_at<'a>(&'a self, i: uint) -> &'a AABB {
        self.bvs.get(i)
    }
}
