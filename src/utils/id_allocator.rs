use slab::Slab;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
/// A unique identifier given by a usize id and a generation number.
pub struct GenerationalId {
    /// The identifier.
    pub id: usize,
    tag: usize,
}

impl GenerationalId {
    /// An identifier that cannot be constructed by `IdAlloc` and that is not equal to any other identifier.
    #[inline]
    pub fn invalid() -> Self {
        GenerationalId {
            id: usize::max_value(),
            tag: 0,
        }
    }

    /// Checks if this identifier is invalid.
    #[inline]
    pub fn is_invalid(&self) -> bool {
        self.id == usize::max_value() && self.tag == 0
    }
}

/// An identifiers allocator.
pub struct IdAllocator {
    generator: Slab<()>,
    generation: usize,
    generation_used: bool,
}

impl IdAllocator {
    /// Creates a new empty id allocator.
    pub fn new() -> Self {
        IdAllocator {
            generator: Slab::new(),
            generation: 1,
            generation_used: false,
        }
    }

    /// The number of allocated identifiers.
    #[inline]
    pub fn len(&self) -> usize {
        self.generator.len()
    }

    /// Allocates a new identifier.
    #[inline]
    pub fn alloc(&mut self) -> GenerationalId {
        let id = self.generator.insert(());
        let tag = self.generation;
        self.generation_used = true;
        GenerationalId { id, tag }
    }

    /// Marks the given identifier as re-usable.
    #[inline]
    pub fn free(&mut self, id: GenerationalId) {
        // FIXME: this is not a robust way of handling this.
        if self.generation_used {
            self.generation = self.generation.wrapping_add(1);
            self.generation_used = false;
        }
        self.generator.remove(id.id)
    }
}
