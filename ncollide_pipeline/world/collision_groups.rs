const SELF_COLLISION: u32 = 1 << 31;
const ALL_GROUPS: u32 = (1 << 30) - 1;
const NO_GROUP: u32 = 0;

/// Groups of collision used to filter which object collide with which other one.
///
/// There are at most 30 groups indexed from 0 to 29 (included). This identifies collidable
/// entities by combining three attributes:
///    * A set of group this structure is member of.
///    * A collision group whitelist.
///    * A collision group blacklist.
///
/// For two entities to collide, they must be member of at least one group part of each-other's
/// whitelists, and must not be part of any blacklisted group. The blacklist always has priority on
/// the whitelist.
///
/// ### Example
/// For example if the object A is such that:
///    * It is part of the groups 1, 3, and 6.
///    * It whitelists the groups 3 and 7.
///    * It blacklists the group 1.
/// Let the object B be such that:
///    * It is part of the groups 1, 3, and 7.
///    * It whitelists the groups 3 and 7.
///    * It does not blacklist anything.
/// For example if the object C is such that:
///    * It is part of the groups 6, 9.
///    * It whitelists the groups 3 and 7.
///    * It does not blacklist anything.
///
/// Then we have:
///    * A and C can collide because A whitelists the group 6 (which C is part of), and,
///    reciprocally, C whitelists the group 3 (which A is part of).
///    * A and B will **not** collide because B is part of the group 1 which is blacklisted by A.
///    * Finally, B and C will **not** collide either because, even if C whitelists the group 3
///    (which B is part of), B does not whitelists the groups 6 nor 9 (which B is part of).
#[derive(RustcEncodable, RustcDecodable, Clone, Debug)]
pub struct CollisionGroups {
    membership: u32,
    whitelist:  u32,
    blacklist:  u32,
}

impl CollisionGroups {
    /// Creates a new `CollisionGroups` that enables collisions with everything except
    /// self-collision.
    #[inline]
    pub fn new() -> CollisionGroups {
        CollisionGroups {
            membership: ALL_GROUPS,
            whitelist:  ALL_GROUPS,
            blacklist:  NO_GROUP
        }
    }

    #[inline]
    fn modify_mask(mask: &mut u32, group_id: u32, add: bool) {
        assert!(group_id < 30, "There are at most 30 groups indexed from 0 to 29 (included).");

        if add {
            *mask = *mask | (1 << group_id)
        }
        else {
            *mask = *mask & !(1 << group_id)
        }
    }

    /// Adds or removes this entity from the given group.
    #[inline]
    pub fn modify_membership(&mut self, group_id: u32, add: bool) {
        CollisionGroups::modify_mask(&mut self.membership, group_id, add);
    }

    /// Adds or removes the given group from this entity whitelist.
    #[inline]
    pub fn modify_whitelist(&mut self, group_id: u32, add: bool) {
        CollisionGroups::modify_mask(&mut self.whitelist, group_id, add);
    }

    /// Adds or removes this entity from the given group.
    #[inline]
    pub fn modify_blacklist(&mut self, group_id: u32, add: bool) {
        CollisionGroups::modify_mask(&mut self.blacklist, group_id, add);
    }

    /// Enables self collision detection.
    #[inline]
    pub fn enable_self_collision(&mut self) {
        self.whitelist = self.whitelist | SELF_COLLISION;
    }

    /// Disables self collision detection.
    #[inline]
    pub fn disable_self_collision(&mut self) {
        self.whitelist = self.whitelist & !SELF_COLLISION;
    }

    #[inline]
    fn is_inside_mask(mask: u32, group_id: u32) -> bool {
        assert!(group_id < 30, "There are at most 30 groups indexed from 0 to 29 (included).");
        mask & (1 << group_id) != 0
    }

    /// Tests if this entity is part of the given group.
    #[inline]
    pub fn is_member_of(&self, group_id: u32) -> bool {
        CollisionGroups::is_inside_mask(self.membership, group_id)
    }

    /// Tests if the given group is whitelisted.
    #[inline]
    pub fn is_group_whitelisted(&self, group_id: u32) -> bool {
        CollisionGroups::is_inside_mask(self.whitelist, group_id)
    }

    /// Tests if the given group is blacklisted.
    #[inline]
    pub fn is_group_blacklisted(&self, group_id: u32) -> bool {
        CollisionGroups::is_inside_mask(self.blacklist, group_id)
    }

    /// Tests whether collisions with a given group is possible.
    ///
    /// Collision is possible if `group_id` is whitelisted but not blacklisted.
    #[inline]
    pub fn can_collide_with(&self, group_id: u32) -> bool {
        !CollisionGroups::is_inside_mask(self.blacklist, group_id) &&
        CollisionGroups::is_inside_mask(self.whitelist, group_id)
    }

    /// Tests whether two collision groups have at least one group in common.
    #[inline]
    pub fn can_collide_with_groups(&self, other: &CollisionGroups) -> bool {
        // FIXME: is there a more bitwise-y way of doing this?
        self.membership  & other.blacklist == 0 &&
        other.membership & self.blacklist  == 0 &&
        self.membership  & other.whitelist != 0 &&
        other.membership & self.whitelist  != 0
    }

    /// Tests whether self-collision is enabled.
    #[inline]
    pub fn can_collide_with_self(&self) -> bool {
        self.whitelist & SELF_COLLISION != 0
    }
}
