use sync::{Arc, Weak, RWLock}

// FIXME: this could be a generic implementation of a binary tree (with a `parent` pointer).
/// A shareable binary tree with a pointer to its parent.
pub struct SurfaceSubdivisionTree<C> {
    data:   C,
    rchild: Option<Arc<RWLock<SurfaceSubdivisionTree<C>>>>,
    lchild: Option<Arc<RWLock<SurfaceSubdivisionTree<C>>>>,
    parent: Option<Weak<RWLock<SurfaceSubdivisionTree<C>>>>
}

impl<C> SurfaceSubdivisionTree<C> {
    /// Changes the righ child of `node`. Fails if the new child is already rooted.
    pub fn set_right_child(node:  &Arc<RWLock<SurfaceSubdivisionTree<C>>,
                           child: Arc<RWLock<SurfaceSubdivisionTree<C>>) {
        assert!(child.read().parent.is_none(), "Cannot add a child that is already rooted.");

        if node.rchild.is_some() {
            SurfaceSubdivisionTree::detach(&mut child);
        }

        node.rchild = Some(child);
    }

    /// Changes the left child of `node`. Fails if the new child is already rooted.
    pub fn set_left_child(node:  &Arc<RWLock<SurfaceSubdivisionTree<C>>,
                          child: Arc<RWLock<SurfaceSubdivisionTree<C>>) {
        assert!(child.read().parent.is_none(), "Cannot add a child that is already rooted.");

        if node.rchild.is_some() {
            SurfaceSubdivisionTree::detach(&mut child);
        }

        node.rchild = Some(child);
    }

    /// Datches the given node from its parent.
    pub fn detach(node: &Arc<RWLock<SurfaceSubdivisionTree<C>>) {
        let bnode = node.write();

        match bnode.parent {
            Some(ref parent) => parent.upgrade().write().remove_child(node),
            None             => { }
        }

        bnode.parent = None;
    }

    /// Reference to the data contained by this node.
    pub fn data<'a>(&'a self) -> &'a C {
        &'a self.data
    }

    /// Mutable reference to the data contained by this node.
    pub fn data_mut<'a>(&'a mut self) -> &'a mut C {
        &'a mut self.data
    }
}
