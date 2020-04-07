use crate::bounding_volume::{self, BoundingVolume, AABB};
use crate::math::Isometry;
use crate::pipeline::broad_phase::BroadPhaseProxyHandle;
use crate::pipeline::narrow_phase::CollisionObjectGraphIndex;
use crate::pipeline::object::CollisionGroups;
use crate::pipeline::object::GeometricQueryType;
use crate::shape::{Shape, ShapeHandle};
use alga::general::RealField;

bitflags! {
    #[derive(Default)]
    /// Flags indicating what changed in a collision object since the last collision world update.
    pub struct CollisionObjectUpdateFlags: u8 {
        /// Indicate that the collision object's position changed.
        const POSITION_CHANGED = 0b00000001;
        /// Indicate that the collision object's predicted position changed.
        const PREDICTED_POSITION_CHANGED = 0b00000010;
        /// Indicate that the collision object's shape changed.
        const SHAPE_CHANGED = 0b000100;
        /// Indicate that the collision object's collision group changed.
        const COLLISION_GROUPS_CHANGED = 0b001000;
        /// Indicate that the collision object's geometric query type changed.
        const QUERY_TYPE_CHANGED = 0b0010000;
    }
}

impl CollisionObjectUpdateFlags {
    /// Checks if the collision object has been changed in a way that justify a broad-phase update.
    pub fn needs_broad_phase_update(&self) -> bool {
        !self.is_empty()
    }

    /// Checks if the collision object has been changed in a way that justify a narrow-phase update.
    pub fn needs_narrow_phase_update(&self) -> bool {
        // The only change that does not trigger an update
        // is a change on predicted position.
        self.intersects(
            Self::POSITION_CHANGED
                | Self::SHAPE_CHANGED
                | Self::COLLISION_GROUPS_CHANGED
                | Self::QUERY_TYPE_CHANGED,
        )
    }

    /// Checks if the collision object has been changed in a way that justify an update of its bounding volume.
    pub fn needs_bounding_volume_update(&self) -> bool {
        // NOTE: the QUERY_TYPE_CHANGED is included here because the
        // prediction margin may have changed.
        self.intersects(Self::POSITION_CHANGED | Self::SHAPE_CHANGED | Self::QUERY_TYPE_CHANGED)
    }

    /// Checks if the collision object has been changed in a way that justify that the broad-phase
    /// recompute all potential proximity pairs for this collision objects.
    pub fn needs_broad_phase_redispatch(&self) -> bool {
        self.intersects(
            Self::SHAPE_CHANGED | Self::COLLISION_GROUPS_CHANGED | Self::QUERY_TYPE_CHANGED,
        )
    }
}

/// Trait implemented by collision objects.
pub trait CollisionObjectRef<N: RealField> {
    /// The interaction graph index of this collision object, if it has been registered into an interaction graph.
    ///
    /// Se the `glue::create_proxies` for more details.
    fn graph_index(&self) -> Option<CollisionObjectGraphIndex>;
    /// The broad-phase proxy handle of this collision object, if it has been registered into a broad-phase.
    ///
    /// Se the `glue::create_proxies` for more details.
    fn proxy_handle(&self) -> Option<BroadPhaseProxyHandle>;
    /// The position of this collision object.
    fn position(&self) -> &Isometry<N>;
    /// The expected position of this collision object in the next updates.
    ///
    /// This is used to enlarge the collision object bounding volume such that at yields more potential interaction pairs.
    /// This is typically needed for CCD (continuous collision detection) to be sure the broad-phase does not miss pential
    /// interactions in-between two discontinuous positions of the collision object.
    fn predicted_position(&self) -> Option<&Isometry<N>>;
    /// The shape of this collision object.
    fn shape(&self) -> &dyn Shape<N>;
    /// The collision groups of this collision object.
    fn collision_groups(&self) -> &CollisionGroups;
    /// The type of geometric queries this collision object is subjected to.
    fn query_type(&self) -> GeometricQueryType<N>;
    /// Flags indicating what changed in this collision object.
    fn update_flags(&self) -> CollisionObjectUpdateFlags;

    /// Computes the AABB of this collision object, ignoring `self.predicted_position()`.
    fn compute_aabb(&self) -> AABB<N> {
        let mut aabb = bounding_volume::aabb(self.shape(), self.position());
        aabb.loosen(self.query_type().query_limit());
        aabb
    }

    /// Computes the swept AABB of this collision object, taking `self.predict_position()` into account.
    ///
    /// Given the AABB of this collision object at the position `self.position()’, and the AABB of
    /// this collision object at the position `self.predicted_position()`, this returns an AABB that
    /// bounds both.
    fn compute_swept_aabb(&self) -> AABB<N> {
        if let Some(predicted_pos) = self.predicted_position() {
            let shape = self.shape();
            let mut aabb1 = bounding_volume::aabb(shape, self.position());
            let mut aabb2 = bounding_volume::aabb(shape, predicted_pos);
            let margin = self.query_type().query_limit();
            aabb1.loosen(margin);
            aabb2.loosen(margin);
            aabb1.merge(&aabb2);
            aabb1
        } else {
            self.compute_aabb()
        }
    }
}

/// The unique identifier of a collision object stored in a `CollisionObjectSlab` structure.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CollisionObjectSlabHandle(pub usize);

impl CollisionObjectSlabHandle {
    /// The unique identifier corresponding to this handle.
    #[inline]
    pub fn uid(&self) -> usize {
        self.0
    }
}

/// A stand-alone object that has a position and a shape.
pub struct CollisionObject<N: RealField, T> {
    proxy_handle: Option<BroadPhaseProxyHandle>,
    graph_index: Option<CollisionObjectGraphIndex>,
    position: Isometry<N>,
    predicted_position: Option<Isometry<N>>,
    shape: ShapeHandle<N>,
    collision_groups: CollisionGroups,
    query_type: GeometricQueryType<N>,
    update_flags: CollisionObjectUpdateFlags,
    data: T,
}

impl<N: RealField, T> CollisionObject<N, T> {
    /// Creates a new collision object.
    pub fn new(
        proxy_handle: Option<BroadPhaseProxyHandle>,
        graph_index: Option<CollisionObjectGraphIndex>,
        position: Isometry<N>,
        shape: ShapeHandle<N>,
        groups: CollisionGroups,
        query_type: GeometricQueryType<N>,
        data: T,
    ) -> CollisionObject<N, T> {
        CollisionObject {
            proxy_handle,
            graph_index,
            position,
            predicted_position: None,
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
    pub fn graph_index(&self) -> Option<CollisionObjectGraphIndex> {
        self.graph_index
    }

    /// Sets the collision object unique but non-stable graph index.
    #[inline]
    pub fn set_graph_index(&mut self, index: Option<CollisionObjectGraphIndex>) {
        self.graph_index = index
    }

    /// Mutable reference to this collision object's update flags.
    pub fn update_flags_mut(&mut self) -> &mut CollisionObjectUpdateFlags {
        &mut self.update_flags
    }

    /// Clears the update flags of this collision object.
    pub fn clear_update_flags(&mut self) {
        self.update_flags = CollisionObjectUpdateFlags::empty()
    }

    /// The collision object's broad phase proxy unique identifier.
    #[inline]
    pub fn proxy_handle(&self) -> Option<BroadPhaseProxyHandle> {
        self.proxy_handle
    }

    /// Set collision object's broad phase proxy unique identifier.
    #[inline]
    pub fn set_proxy_handle(&mut self, handle: Option<BroadPhaseProxyHandle>) {
        self.proxy_handle = handle
    }

    /// The collision object position.
    #[inline]
    pub fn position(&self) -> &Isometry<N> {
        &self.position
    }

    /// The predicted collision object position.
    #[inline]
    pub fn predicted_position(&self) -> Option<&Isometry<N>> {
        self.predicted_position.as_ref()
    }

    /// Sets the position of the collision object and resets the predicted position to None.
    #[inline]
    pub fn set_position(&mut self, pos: Isometry<N>) {
        self.update_flags |= CollisionObjectUpdateFlags::POSITION_CHANGED;
        self.update_flags |= CollisionObjectUpdateFlags::PREDICTED_POSITION_CHANGED;
        self.position = pos;
        self.predicted_position = None;
    }

    /// Sets the position of the collision object and resets the predicted position.
    #[inline]
    pub fn set_position_with_prediction(&mut self, pos: Isometry<N>, prediction: Isometry<N>) {
        self.update_flags |= CollisionObjectUpdateFlags::POSITION_CHANGED;
        self.update_flags |= CollisionObjectUpdateFlags::PREDICTED_POSITION_CHANGED;
        self.position = pos;
        self.predicted_position = Some(prediction);
    }

    /// Sets the predicted position of the collision object.
    #[inline]
    pub fn set_predicted_position(&mut self, pos: Option<Isometry<N>>) {
        self.update_flags |= CollisionObjectUpdateFlags::PREDICTED_POSITION_CHANGED;
        self.predicted_position = pos;
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

    /// Sets the collision groups of this collision object.
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

impl<N: RealField, T> CollisionObjectRef<N> for CollisionObject<N, T> {
    fn graph_index(&self) -> Option<CollisionObjectGraphIndex> {
        self.graph_index()
    }

    fn proxy_handle(&self) -> Option<BroadPhaseProxyHandle> {
        self.proxy_handle()
    }

    fn position(&self) -> &Isometry<N> {
        self.position()
    }

    fn predicted_position(&self) -> Option<&Isometry<N>> {
        self.predicted_position()
    }

    fn shape(&self) -> &dyn Shape<N> {
        self.shape().as_ref()
    }

    fn collision_groups(&self) -> &CollisionGroups {
        self.collision_groups()
    }

    fn query_type(&self) -> GeometricQueryType<N> {
        self.query_type()
    }

    fn update_flags(&self) -> CollisionObjectUpdateFlags {
        self.update_flags
    }
}
