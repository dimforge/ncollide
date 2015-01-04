use na::{Pnt3, Vec2, Vec3};
use na;
use utils;
use parametric::ParametricSurface;
use procedural::TriMesh;
use procedural;
use math::Scalar;

/// Meshing algorithm that uniformly triangulates the parametric space.
pub fn parametric_surface_uniform<N, S>(s: &S, usubdivs: uint, vsubdivs: uint)
                                        -> TriMesh<N, Pnt3<N>, Vec3<N>>
    where N: Scalar,
          S: ParametricSurface<N, Pnt3<N>, Vec3<N>> {
    assert!(usubdivs > 0);
    assert!(vsubdivs > 0);

    let mut surface = procedural::unit_quad::<N, Pnt3<N>, Vec3<N>>(usubdivs, vsubdivs);

    {
        let uvs     = surface.uvs.as_ref().unwrap().as_slice();
        let normals = surface.normals.as_mut().unwrap().as_mut_slice();
        let coords  = surface.coords.as_mut_slice();

        for j in range(0, vsubdivs + 1) {
            for i in range(0, usubdivs + 1) {
                let id            = i + j * (usubdivs + 1);
                let (s, s_u, s_v) = s.at_u_v(uvs[id].x, uvs[id].y);

                normals[id] = na::cross(&s_u, &s_v);
                coords[id]  = s;
            }
        }
    }

    surface
}

/// Uniformly triangulates the parametric space.
///
/// The triangulation step is chosen to guarantee that the maximum pointwise distance between the
/// piecewise linear approximation and the parametric surface is bellow `error`.
///
/// See: "Surface algorithms using bounds on derivatives", D. Filip, R. Magedson, R. Markot
pub fn parametric_surface_uniform_with_distance_error<N, S>(s: &S, error: N) -> TriMesh<N, Pnt3<N>, Vec3<N>>
    where N: Scalar,
          S: ParametricSurface<N, Pnt3<N>, Vec3<N>> {
    let mut ms: [N; 3] = [ na::zero(), na::zero(), na::zero() ];

    for (mi, &(i, j)) in [ (2u, 0u), (1, 1), (0, 2) ].iter().enumerate() {
        let mut eval_dij    = |arg: &Vec2<N>| -na::sqnorm(&s.at_uv_nk(arg.x, arg.y, i, j));
        let mut eval_sq_dij = |arg: &Vec2<N>| {
            let du0v0 = s.at_uv_nk(arg.x, arg.y, i + 0, j + 0);
            let du1v0 = s.at_uv_nk(arg.x, arg.y, i + 1, j + 0);
            let du0v1 = s.at_uv_nk(arg.x, arg.y, i + 0, j + 1);

            let grad = Vec2::new(
                na::dot(&du1v0, &du0v0) * na::cast(2.0f64),
                na::dot(&du0v1, &du0v0) * na::cast(2.0f64)
                );

            -grad
        };

        let (_, m) = utils::minimize_with_bfgs(
            10, 1000,
            &na::zero(), &na::one(),
            &mut eval_dij, &mut eval_sq_dij);

        ms[mi] = (-m).sqrt();
    }

    let n;
    let m;

    let _2: N = na::cast(2.0f64);
    let _8: N = na::cast(8.0f64);

    if ms[0] > na::zero() {
        if ms[2] > na::zero() {
            let k  = ms[0] / ms[2];
            let kk = k * k;
            m = ((ms[0] + _2 * k * ms[1] + kk * ms[2]) / (_8 * k * k * error)).sqrt();
            n = k * m;
        }
        else {
            m = na::one();
            n = ms[1] + (ms[1] * ms[1] + _8 * error * ms[0]);
        }
    }
    else {
        if ms[2] > na::zero() {
            n = na::one();
            m = ms[1] + (ms[1] * ms[1] + _8 * error * ms[0]);
        }
        else {
            m = ((ms[0] + _2 *  ms[1] + ms[2]) / (_8 * error)).sqrt();
            n = m;
        }
    }

    let _1: N = na::one();

    // FIXME: round instead of `as`?
    parametric_surface_uniform(s, n.ceil().to_uint().unwrap(), m.ceil().to_uint().unwrap())
}
