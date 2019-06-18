use na::RealField;
use crate::pipeline::broad_phase::BroadPhasePairFilter;
use crate::pipeline::object::CollisionObjectRef;

const SELF_COLLISION: u32 = 1 << 31;
const ALL_GROUPS: u32 = (1 << 30) - 1;
const NO_GROUP: u32 = 0;

/// Groups of collision used to filter which object interact with which other one.
///
/// There are at most 30 groups indexed from 0 to 29 (included). This identifies collidable
/// entities by combining three attributes:
///    * A set of group this structure is member of.
///    * A collision group whitelist.
///    * A collision group blacklist.
///
/// For two entities to interact, they must be member of at least one group part of each-other's
/// whitelists, and must not be part of any blacklisted group. The blacklist always has priority on
/// the whitelist.
///
/// ### Example
/// For example if the object A is such that:
///    * It is part of the groups 1, 3, and 6.
///    * It whitelists the groups 6 and 7.
///    * It blacklists the group 1.
/// Let the object B be such that:
///    * It is part of the groups 1, 3, and 7.
///    * It whitelists the groups 3 and 7.
///    * It does not blacklist anything.
/// For example if the object C is such that:
///    * It is part of the groups 6 and 9.
///    * It whitelists the groups 3 and 7.
///    * It does not blacklist anything.
///
/// Then we have:
///    * A and C can interact because A whitelists the group 6 (which C is part of), and,
///    reciprocally, C whitelists the group 3 (which A is part of).
///    * A and B will **not** interact because B is part of the group 1 which is blacklisted by A.
///    * Finally, B and C will **not** interact either because, even if C whitelists the group 3
///    (which B is part of), B does not whitelists the groups 6 nor 9 (which B is part of).
#[derive(Clone, Debug, Copy)]
pub struct CollisionGroups {
    membership: u32,
    whitelist: u32,
    blacklist: u32,
}

impl CollisionGroups {
    /// Creates a new `CollisionGroups` that enables interactions with everything except
    /// self-interaction.
    #[inline]
    pub fn new() -> CollisionGroups {
        CollisionGroups {
            membership: ALL_GROUPS,
            whitelist: ALL_GROUPS,
            blacklist: NO_GROUP,
        }
    }

    /// Returns a copy of this object, updated with a new set of membership groups.
    ///
    /// # Examples
    ///
    /// ```.ignore
    /// const GROUP_A: usize = 0;
    /// const GROUP_B: usize = 29;
    /// let groups = CollisionGroups::new().with_membership(&[GROUP_A, GROUP_B]);
    /// assert!(groups.is_member_of(GROUP_A));
    /// assert!(groups.is_member_of(GROUP_B));
    /// ```
    #[inline]
    pub fn with_membership(mut self, groups: &[usize]) -> CollisionGroups {
        CollisionGroups::set_mask(&mut self.membership, groups);
        self
    }

    /// Returns a copy of this object, updated with a new set of whitelisted groups.
    ///
    /// # Examples
    ///
    /// ```.ignore
    /// const GROUP_A: usize = 0;
    /// const GROUP_B: usize = 29;
    /// let group_a = CollisionGroups::new().with_whitelist(&[GROUP_B]);
    /// assert!(!group_a.is_group_whitelisted(GROUP_A));
    /// assert!(group_a.is_group_whitelisted(GROUP_B));
    /// ```
    #[inline]
    pub fn with_whitelist(mut self, groups: &[usize]) -> CollisionGroups {
        CollisionGroups::set_mask(&mut self.whitelist, groups);
        self
    }

    /// Returns a copy of this object, updated with a new set of blacklisted groups.
    ///
    /// # Examples
    ///
    /// ```.ignore
    /// const GROUP_A: usize = 0;
    /// const GROUP_B: usize = 29;
    /// let group_a = CollisionGroups::new().with_blacklist(&[GROUP_B]);
    /// assert!(!group_a.is_group_blacklisted(GROUP_A));
    /// assert!(group_a.is_group_blacklisted(GROUP_B));
    /// ```
    #[inline]
    pub fn with_blacklist(mut self, groups: &[usize]) -> CollisionGroups {
        CollisionGroups::set_mask(&mut self.blacklist, groups);
        self
    }

    /// The maximum allowed group identifier.
    #[inline]
    pub fn max_group_id() -> usize {
        29
    }

    #[inline]
    fn modify_mask(mask: &mut u32, group_id: usize, add: bool) {
        assert!(
            group_id < 30,
            "There are at most 30 groups indexed from 0 to 29 (included)."
        );

        if add {
            *mask = *mask | (1 << group_id)
        } else {
            *mask = *mask & !(1 << group_id)
        }
    }

    #[inline]
    fn set_mask(mask: &mut u32, groups: &[usize]) {
        *mask = 0;
        for g in groups.iter() {
            CollisionGroups::modify_mask(mask, *g, true);
        }
    }

    /// Adds or removes this entity from the given group.
    #[inline]
    pub fn modify_membership(&mut self, group_id: usize, add: bool) {
        CollisionGroups::modify_mask(&mut self.membership, group_id, add);
    }

    /// Adds or removes the given group from this entity whitelist.
    #[inline]
    pub fn modify_whitelist(&mut self, group_id: usize, add: bool) {
        CollisionGroups::modify_mask(&mut self.whitelist, group_id, add);
    }

    /// Adds or removes this entity from the given group.
    #[inline]
    pub fn modify_blacklist(&mut self, group_id: usize, add: bool) {
        CollisionGroups::modify_mask(&mut self.blacklist, group_id, add);
    }

    /// Make this object member of the given groups only.
    #[inline]
    pub fn set_membership(&mut self, groups: &[usize]) {
        CollisionGroups::set_mask(&mut self.membership, groups);
    }

    /// Whitelists the given groups only (others will be un-whitelisted).
    #[inline]
    pub fn set_whitelist(&mut self, groups: &[usize]) {
        CollisionGroups::set_mask(&mut self.whitelist, groups);
    }

    /// Blacklists the given groups only (others will be un-blacklisted).
    #[inline]
    pub fn set_blacklist(&mut self, groups: &[usize]) {
        CollisionGroups::set_mask(&mut self.blacklist, groups);
    }

    /// Copies the membership of another collision groups.
    #[inline]
    pub fn copy_membership(&mut self, other: &CollisionGroups) {
        self.membership = other.membership
    }

    /// Copies the whitelist of another collision groups.
    #[inline]
    pub fn copy_whitelist(&mut self, other: &CollisionGroups) {
        self.whitelist = other.whitelist
    }

    /// Copies the blacklist of another collision groups.
    #[inline]
    pub fn copy_blacklist(&mut self, other: &CollisionGroups) {
        self.blacklist = other.blacklist
    }

    /// Allows the object to interact with itself.
    #[inline]
    pub fn enable_self_interaction(&mut self) {
        self.whitelist = self.whitelist | SELF_COLLISION;
    }

    /// Prevents the object from interacting with itself.
    #[inline]
    pub fn disable_self_interaction(&mut self) {
        self.whitelist = self.whitelist & !SELF_COLLISION;
    }

    #[inline]
    fn is_inside_mask(mask: u32, group_id: usize) -> bool {
        assert!(
            group_id < 30,
            "There are at most 30 groups indexed from 0 to 29 (included)."
        );
        mask & (1 << group_id) != 0
    }

    /// Tests if this entity is part of the given group.
    #[inline]
    pub fn is_member_of(&self, group_id: usize) -> bool {
        CollisionGroups::is_inside_mask(self.membership, group_id)
    }

    /// Tests if the given group is whitelisted.
    #[inline]
    pub fn is_group_whitelisted(&self, group_id: usize) -> bool {
        CollisionGroups::is_inside_mask(self.whitelist, group_id)
    }

    /// Tests if the given group is blacklisted.
    #[inline]
    pub fn is_group_blacklisted(&self, group_id: usize) -> bool {
        CollisionGroups::is_inside_mask(self.blacklist, group_id)
    }

    /// Tests whether interactions with a given group is possible.
    ///
    /// Collision is possible if `group_id` is whitelisted but not blacklisted.
    #[inline]
    pub fn can_interact_with(&self, group_id: usize) -> bool {
        !CollisionGroups::is_inside_mask(self.blacklist, group_id)
            && CollisionGroups::is_inside_mask(self.whitelist, group_id)
    }

    /// Tests whether two collision groups have at least one group in common.
    #[inline]
    pub fn can_interact_with_groups(&self, other: &CollisionGroups) -> bool {
        // FIXME: is there a more bitwise-y way of doing this?
        self.membership & other.blacklist == 0
            && other.membership & self.blacklist == 0
            && self.membership & other.whitelist != 0
            && other.membership & self.whitelist != 0
    }

    /// Tests whether self-interaction is enabled.
    #[inline]
    pub fn can_interact_with_self(&self) -> bool {
        self.whitelist & SELF_COLLISION != 0
    }
}

impl Default for CollisionGroups {
    #[inline]
    fn default() -> Self {
        CollisionGroups::new()
    }
}


/// A collision filter based collision groups.
pub struct CollisionGroupsPairFilter;

impl CollisionGroupsPairFilter {
    /// Creates a new collision filter based collision groups.
    #[inline]
    pub fn new() -> CollisionGroupsPairFilter {
        CollisionGroupsPairFilter
    }
}

impl<'a, N: RealField, Object: CollisionObjectRef<'a, N>> BroadPhasePairFilter<N, Object> for CollisionGroupsPairFilter {
    fn is_pair_valid(&self, co1: Object, co2: Object) -> bool {
        if co1.is_same_as(co2) {
            co1.collision_groups().can_interact_with_self()
        } else {
            co1.collision_groups()
                .can_interact_with_groups(co2.collision_groups())
        }
    }
}