use alga::general::RealField;
use crate::math::Isometry;
use crate::pipeline::broad_phase::BroadPhaseProxyHandle;
use crate::pipeline::narrow_phase::CollisionObjectGraphIndex;
use crate::pipeline::object::CollisionGroups;
use crate::query::ContactPrediction;
use crate::shape::{Shape, ShapeHandle};
use crate::bounding_volume::{self, BoundingVolume, AABB};
use crate::pipeline::object::GeometricQueryType;


bitflags! {
    #[derive(Default)]
    pub struct CollisionObjectUpdateFlags: u8 {
        const POSITION_CHANGED = 0b00000001;
        const SHAPE_CHANGED = 0b000010;
        const COLLISION_GROUPS_CHANGED = 0b000100;
        const QUERY_TYPE_CHANGED = 0b0001000;
    }
}

impl CollisionObjectUpdateFlags {
    pub fn needs_broad_phase_update(&self) -> bool {
        !self.is_empty()
    }

    pub fn needs_narrow_phase_update(&self) -> bool {
        !self.is_empty()
    }

    pub fn needs_bounding_volume_update(&self) -> bool {
        // NOTE: the QUERY_TYPE_CHANGED is included here because the
        // prediction margin may have changed.
        self.intersects(Self::POSITION_CHANGED | Self::SHAPE_CHANGED | Self::QUERY_TYPE_CHANGED)
    }

    pub fn needs_broad_phase_redispatch(&self) -> bool {
        self.intersects(Self::SHAPE_CHANGED | Self::COLLISION_GROUPS_CHANGED | Self::QUERY_TYPE_CHANGED)
    }
}

pub trait CollisionObjectRef<'a, N: RealField>: Copy {
    fn graph_index(self) -> CollisionObjectGraphIndex;
    fn proxy_handle(self) -> BroadPhaseProxyHandle;
    fn position(self) -> &'a Isometry<N>;
    fn shape(self) -> &'a Shape<N>;
    fn collision_groups(self) -> &'a CollisionGroups;
    fn query_type(self) -> GeometricQueryType<N>;
    fn update_flags(self) -> CollisionObjectUpdateFlags;

    fn compute_aabb(self) -> AABB<N> {
        let mut aabb = bounding_volume::aabb(self.shape(), self.position());
        aabb.loosen(self.query_type().query_limit());
        aabb
    }

    fn compute_swept_aabb(self, predicted_pos: &Isometry<N>) -> AABB<N> {
        let shape = self.shape();
        let mut aabb1 = bounding_volume::aabb(shape, self.position());
        let mut aabb2 = bounding_volume::aabb(shape, predicted_pos);
        let margin = self.query_type().query_limit();
        aabb1.loosen(margin);
        aabb2.loosen(margin);
        aabb1.merge(&aabb2);
        aabb1
    }
}

/// The unique identifier of a collision object.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CollisionObjectSlabHandle(pub usize);

impl CollisionObjectSlabHandle {
    #[inline]
    pub(crate) fn invalid() -> Self {
        CollisionObjectSlabHandle(usize::max_value())
    }

    /// The unique identifier corresponding to this handle.
    #[inline]
    pub fn uid(&self) -> usize {
        self.0
    }
}

/// A stand-alone object that has a position and a shape.
pub struct CollisionObject<N: RealField, T> {
    proxy_handle: BroadPhaseProxyHandle,
    graph_index: CollisionObjectGraphIndex,
    position: Isometry<N>,
    shape: ShapeHandle<N>,
    collision_groups: CollisionGroups,
    query_type: GeometricQueryType<N>,
    update_flags: CollisionObjectUpdateFlags,
    data: T,
}

impl<N: RealField, T> CollisionObject<N, T> {
    /// Creates a new collision object.
    pub fn new(
        proxy_handle: BroadPhaseProxyHandle,
        graph_index: CollisionObjectGraphIndex,
        position: Isometry<N>,
        shape: ShapeHandle<N>,
        groups: CollisionGroups,
        query_type: GeometricQueryType<N>,
        data: T,
    ) -> CollisionObject<N, T>
    {
        CollisionObject {
            proxy_handle,
            graph_index,
            position,
            shape,
            collision_groups: groups,
            data,
            query_type,
            update_flags: CollisionObjectUpdateFlags::all(),
        }
    }

    /// The collision object non-stable graph index.
    ///
    /// This index may change whenever a collision object is removed from the world.
    #[inline]
    pub fn graph_index(&self) -> CollisionObjectGraphIndex {
        self.graph_index
    }

    /// Sets the collision object unique but non-stable graph index.
    #[inline]
    pub fn set_graph_index(&mut self, index: CollisionObjectGraphIndex) {
        self.graph_index = index
    }

    pub fn update_flags_mut(&mut self) -> &mut CollisionObjectUpdateFlags {
        &mut self.update_flags
    }

    pub fn clear_update_flags(&mut self) {
        self.update_flags = CollisionObjectUpdateFlags::empty()
    }

    /// The collision object's broad phase proxy unique identifier.
    #[inline]
    pub fn proxy_handle(&self) -> BroadPhaseProxyHandle {
        self.proxy_handle
    }

    /// The collision object position.
    #[inline]
    pub fn position(&self) -> &Isometry<N> {
        &self.position
    }

    /// Sets the position of the collision object.
    #[inline]
    pub fn set_position(&mut self, pos: Isometry<N>) {
        self.update_flags |= CollisionObjectUpdateFlags::POSITION_CHANGED;
        self.position = pos
    }

    /// Deforms the underlying shape if possible.
    ///
    /// Panics if the shape is not deformable.
    #[inline]
    pub fn set_deformations(&mut self, coords: &[N]) {
        self.update_flags |= CollisionObjectUpdateFlags::POSITION_CHANGED;
        self.shape
            .make_mut()
            .as_deformable_shape_mut()
            .expect("Attempting to deform a non-deformable shape.")
            .set_deformations(coords)
    }

    /// The collision object shape.
    #[inline]
    pub fn shape(&self) -> &ShapeHandle<N> {
        &self.shape
    }

    /// Set the collision object shape.
    #[inline]
    pub fn set_shape(&mut self, shape: ShapeHandle<N>) {
        self.update_flags |= CollisionObjectUpdateFlags::SHAPE_CHANGED;
        self.shape = shape
    }

    /// The collision groups of the collision object.
    #[inline]
    pub fn collision_groups(&self) -> &CollisionGroups {
        &self.collision_groups
    }

    #[inline]
    pub fn set_collision_groups(&mut self, groups: CollisionGroups) {
        self.update_flags |= CollisionObjectUpdateFlags::COLLISION_GROUPS_CHANGED;
        self.collision_groups = groups
    }

    /// The kind of queries this collision object is expected to .
    #[inline]
    pub fn query_type(&self) -> GeometricQueryType<N> {
        self.query_type
    }

    /// Sets the `GeometricQueryType` of the collision object.
    /// Use `CollisionWorld::set_query_type` to use this method.
    #[inline]
    pub fn set_query_type(&mut self, query_type: GeometricQueryType<N>) {
        self.update_flags |= CollisionObjectUpdateFlags::QUERY_TYPE_CHANGED;
        self.query_type = query_type;
    }

    /// Reference to the user-defined data associated to this object.
    #[inline]
    pub fn data(&self) -> &T {
        &self.data
    }

    /// Mutable reference to the user-defined data associated to this object.
    #[inline]
    pub fn data_mut(&mut self) -> &mut T {
        &mut self.data
    }
}


impl<'a, N: RealField, T> CollisionObjectRef<'a, N> for &'a CollisionObject<N, T> {
    fn graph_index(self) -> CollisionObjectGraphIndex {
        self.graph_index()
    }

    fn proxy_handle(self) -> BroadPhaseProxyHandle {
        self.proxy_handle()
    }

    fn position(self) -> &'a Isometry<N> {
        self.position()
    }

    fn shape(self) -> &'a Shape<N> {
        self.shape().as_ref()
    }

    fn collision_groups(self) -> &'a CollisionGroups {
        self.collision_groups()
    }

    fn query_type(self) -> GeometricQueryType<N> {
        self.query_type()
    }

    fn update_flags(self) -> CollisionObjectUpdateFlags {
        self.update_flags
    }
}
