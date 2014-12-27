const SELF_COLLISION: u32 = 1 << 31;
const ALL_GROUPS: u32 = (1 << 30) - 1;

/// 30 groups of collision used to filter which object collide with which other one.
///
/// If two objects have at least one collision group in common, they can collide. Otherwise, they
/// cannot.
#[deriving(RustcEncodable, RustcDecodable, Clone, Show)]
pub struct CollisionGroups {
    tag: u32
}

impl CollisionGroups {
    /// Creates a new `CollisionGroups` that enables collisions with everything except
    /// self-collision.
    #[inline]
    pub fn new() -> CollisionGroups {
        CollisionGroups {
            tag: ALL_GROUPS
        }
    }

    /// Creates a new `CollisionGroups` that enables collisions with the given groups.
    ///
    /// This does not enable self-collision.
    #[inline]
    pub fn new_with_groups(groups: &[uint]) -> CollisionGroups {
        let mut res = CollisionGroups { tag: 0 };

        for group in groups.iter() {
            res.enable_collisions_with_group(*group)
        }

        res
    }

    /// Creates a new `CollisionGroups` that enables collisions with every group but the given
    /// ones.
    ///
    /// This does not enable self-collision.
    #[inline]
    pub fn new_without_groups(groups: &[uint]) -> CollisionGroups {
        let mut res = CollisionGroups { tag: ALL_GROUPS };

        for group in groups.iter() {
            res.disable_collisions_with_group(*group)
        }

        res
    }

    /// Enables collision with all groups.
    ///
    /// This does not affect self-collision.
    #[inline]
    pub fn enable_collisions_with_all_groups(&mut self) {
        self.tag = self.tag | ALL_GROUPS;
    }

    /// Disables collision with all groups.
    ///
    /// This does not affect self-collision.
    #[inline]
    pub fn disable_collisions_with_all_groups(&mut self) {
        self.tag = self.tag & !ALL_GROUPS;
    }

    /// Enable collision detection with a specific group.
    #[inline]
    pub fn enable_collisions_with_group(&mut self, group_id: uint) {
        assert!(group_id < 30, "There are at most 30 groups indexed from 0 to 29 (included).");

        self.tag = self.tag | (1 << group_id);
    }

    /// Disables collision detection with a specific group.
    #[inline]
    pub fn disable_collisions_with_group(&mut self, group_id: uint) {
        assert!(group_id < 30, "There are at most 30 groups indexed from 0 to 29 (included).");

        self.tag = self.tag & !(1 << group_id);
    }

    /// Enables self collision detection.
    #[inline]
    pub fn enable_self_collision(&mut self) {
        self.tag = self.tag | SELF_COLLISION;
    }

    /// Disables self collision detection.
    #[inline]
    pub fn disable_self_collision(&mut self) {
        self.tag = self.tag & !SELF_COLLISION;
    }

    /// Tests whether collisions with a given group is enabled.
    #[inline]
    pub fn can_collide_with_group(&self, group_id: uint) -> bool {
        assert!(group_id < 30, "There are at most 30 groups indexed from 0 to 29 (included).");
        self.tag & (1 << group_id) != 0
    }

    /// Tests whether two collision groups have at least one group in common.
    #[inline]
    pub fn can_collide_with_groups(&self, other: &CollisionGroups) -> bool {
        (self.tag & other.tag & ALL_GROUPS) != 0
    }

    /// Tests whether self-collision is enabled.
    #[inline]
    pub fn can_collide_with_self(&self) -> bool {
        self.tag & SELF_COLLISION != 0
    }
}
