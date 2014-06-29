use sync::{Arc, RWLock};
use math::{Scalar, Vect, Matrix};
use geom::BezierSurface;
use narrow::{CollisionDetector, Contact};
use narrow::surface_selector::SurfaceSelector;
use narrow::surface_subdivision_tree::{SurfaceSubdivisionTreeRef, SurfaceSubdivisionTreeCache};


/// A collision detector between two bézier surfaces.
pub struct BezierSurfaceBezierSurface<S, D> {
    cache:      Arc<RWLock<SurfaceSubdivisionTreeCache<D>>>,
    tree:       Option<SurfaceSubdivisionTreeRef<D>>,
    selector:   S,
    prediction: Scalar,
    contacts:   Vec<Contact>,
    points:     Vec<Vect>,
    timestamp:  uint
}

impl<S: Clone, D: Send> Clone for BezierSurfaceBezierSurface<S, D> {
    fn clone(&self) -> BezierSurfaceBezierSurface<S, D> {
        BezierSurfaceBezierSurface {
            cache:      self.cache.clone(),
            tree:       self.tree.clone(),
            selector:   self.selector.clone(),
            prediction: self.prediction.clone(),
            contacts:   self.contacts.clone(),
            points:     self.points.clone(),
            timestamp:  self.timestamp.clone()
        }
    }
}

impl<S: SurfaceSelector<D>, D> BezierSurfaceBezierSurface<S, D> {
    /// Creates a new collision detector between two bézier surfaces.
    pub fn new(selector: S,
               prediction: Scalar,
               cache:      Arc<RWLock<SurfaceSubdivisionTreeCache<D>>>)
               -> BezierSurfaceBezierSurface<S, D> {
        BezierSurfaceBezierSurface {
            cache:      cache,
            tree:       None,
            selector:   selector,
            prediction: prediction,
            contacts:   Vec::new(),
            points:     Vec::new(),
            timestamp:  0 // 0 because the tree starts at 1.
        }
    }
}

impl<S: SurfaceSelector<D>, D>
CollisionDetector<BezierSurface, BezierSurface> for BezierSurfaceBezierSurface<S, D> {
    fn update(&mut self, ma: &Matrix, a: &BezierSurface, mb: &Matrix, b: &BezierSurface) {
        self.points.clear();
        self.contacts.clear();
        self.selector.set_max_lmd(self.prediction);


        // FIXME
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.contacts.len()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact>) {
        out_colls.push_all(self.contacts.as_slice())
    }

    #[inline]
    fn toi(_: Option<BezierSurfaceBezierSurface<S, D>>,
           _: &Matrix,
           _: &Vect,
           _: &Scalar,
           _: &BezierSurface,
           _: &Matrix,
           _: &BezierSurface)
           -> Option<Scalar> {
        None
    }
}
