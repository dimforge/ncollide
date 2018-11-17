#[cfg(feature = "dim3")]
use super::TriMesh;
use math::Point;
use na::{self, Real};
use std::iter;
use std::ptr;

// De-Casteljau algorithm.
// Evaluates the bezier curve with control points `control_points`.
#[doc(hidden)]
pub fn bezier_curve_at<N: Real>(
    control_points: &[Point<N>],
    t: N,
    cache: &mut Vec<Point<N>>,
) -> Point<N>
{
    if control_points.len() > cache.len() {
        let diff = control_points.len() - cache.len();
        cache.extend(iter::repeat(Point::origin()).take(diff))
    }

    let cache = &mut cache[..];

    let _1: N = na::convert(1.0);
    let t_1 = _1 - t;

    // XXX: not good if the objects are not POD.
    unsafe {
        ptr::copy_nonoverlapping(
            control_points.as_ptr(),
            cache.as_mut_ptr(),
            control_points.len(),
        );
    }

    for i in 1usize..control_points.len() {
        for j in 0usize..control_points.len() - i {
            cache[j] = cache[j] * t_1 + cache[j + 1].coords * t;
        }
    }

    cache[0].clone()
}

// Evaluates the bezier curve with control points `control_points`.
#[cfg(feature = "dim3")]
#[doc(hidden)]
pub fn bezier_surface_at<N: Real>(
    control_points: &[Point<N>],
    nupoints: usize,
    nvpoints: usize,
    u: N,
    v: N,
    ucache: &mut Vec<Point<N>>,
    vcache: &mut Vec<Point<N>>,
) -> Point<N>
where
    N: Real,
{
    if vcache.len() < nvpoints {
        let diff = nvpoints - vcache.len();
        vcache.extend(iter::repeat(Point::origin()).take(diff));
    }

    // FIXME: start with u or v, depending on which dimension has more control points.
    let vcache = &mut vcache[..];

    for i in 0..nvpoints {
        let start = i * nupoints;
        let end = start + nupoints;

        vcache[i] = bezier_curve_at(&control_points[start..end], u, ucache);
    }

    bezier_curve_at(&vcache[0..nvpoints], v, ucache)
}

/// Given a set of control points, generates a (non-rational) Bezier curve.
pub fn bezier_curve<N: Real>(control_points: &[Point<N>], nsubdivs: usize) -> Vec<Point<N>> {
    let mut coords = Vec::with_capacity(nsubdivs);
    let mut cache = Vec::new();
    let tstep = na::convert(1.0 / (nsubdivs as f64));
    let mut t = na::zero::<N>();

    while t <= na::one() {
        coords.push(bezier_curve_at(control_points, t, &mut cache));
        t = t + tstep;
    }

    coords
}

/// Given a set of control points, generates a (non-rational) Bezier surface.
#[cfg(feature = "dim3")]
pub fn bezier_surface<N: Real>(
    control_points: &[Point<N>],
    nupoints: usize,
    nvpoints: usize,
    usubdivs: usize,
    vsubdivs: usize,
) -> TriMesh<N>
where
    N: Real,
{
    assert!(nupoints * nvpoints == control_points.len());

    let mut surface = super::unit_quad(usubdivs, vsubdivs);

    {
        let uvs = &surface.uvs.as_ref().unwrap()[..];
        let coords = &mut surface.coords[..];

        let mut ucache = Vec::new();
        let mut vcache = Vec::new();

        for j in 0..vsubdivs + 1 {
            for i in 0..usubdivs + 1 {
                let id = i + j * (usubdivs + 1);
                coords[id] = bezier_surface_at(
                    control_points,
                    nupoints,
                    nvpoints,
                    uvs[id].x,
                    uvs[id].y,
                    &mut ucache,
                    &mut vcache,
                )
            }
        }

        // XXX: compute the normals manually.
        surface.normals = None;
    }

    surface
}
