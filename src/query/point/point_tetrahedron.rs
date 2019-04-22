use crate::math::{Isometry, Point, Vector};
use na::{self, RealField};
use crate::query::{PointProjection, PointQuery, PointQueryWithLocation};
use crate::shape::{FeatureId, Tetrahedron, TetrahedronPointLocation};

impl<N: RealField> PointQuery<N> for Tetrahedron<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        let (projection, _) = self.project_point_with_location(m, pt, solid);
        projection
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        let (proj, loc) = self.project_point_with_location(m, pt, false);
        let feature = match loc {
            TetrahedronPointLocation::OnVertex(i) => FeatureId::Vertex(i),
            TetrahedronPointLocation::OnEdge(i, _) => FeatureId::Edge(i),
            TetrahedronPointLocation::OnFace(i, _) => FeatureId::Face(i),
            TetrahedronPointLocation::OnSolid => unreachable!(),
        };

        (proj, feature)
    }
}

impl<N: RealField> PointQueryWithLocation<N> for Tetrahedron<N> {
    type Location = TetrahedronPointLocation<N>;

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
        solid: bool,
    ) -> (PointProjection<N>, Self::Location)
    {
        let p = m.inverse_transform_point(pt);

        let ab = *self.b() - *self.a();
        let ac = *self.c() - *self.a();
        let ad = *self.d() - *self.a();
        let ap = p - *self.a();

        /*
         * Voronoï regions of vertices.
         */
        let ap_ab = ap.dot(&ab);
        let ap_ac = ap.dot(&ac);
        let ap_ad = ap.dot(&ad);

        let _0: N = na::zero();

        if ap_ab <= _0 && ap_ac <= _0 && ap_ad <= _0 {
            // Voronoï region of `a`.
            let proj = PointProjection::new(false, m * self.a());
            return (proj, TetrahedronPointLocation::OnVertex(0));
        }

        let bc = *self.c() - *self.b();
        let bd = *self.d() - *self.b();
        let bp = p - *self.b();

        let bp_bc = bp.dot(&bc);
        let bp_bd = bp.dot(&bd);
        let bp_ab = bp.dot(&ab);

        if bp_bc <= _0 && bp_bd <= _0 && bp_ab >= _0 {
            // Voronoï region of `b`.
            let proj = PointProjection::new(false, m * self.b());
            return (proj, TetrahedronPointLocation::OnVertex(1));
        }

        let cd = *self.d() - *self.c();
        let cp = p - *self.c();

        let cp_ac = cp.dot(&ac);
        let cp_bc = cp.dot(&bc);
        let cp_cd = cp.dot(&cd);

        if cp_cd <= _0 && cp_bc >= _0 && cp_ac >= _0 {
            // Voronoï region of `c`.
            let proj = PointProjection::new(false, m * self.c());
            return (proj, TetrahedronPointLocation::OnVertex(2));
        }

        let dp = p - *self.d();

        let dp_cd = dp.dot(&cd);
        let dp_bd = dp.dot(&bd);
        let dp_ad = dp.dot(&ad);

        if dp_ad >= _0 && dp_bd >= _0 && dp_cd >= _0 {
            // Voronoï region of `d`.
            let proj = PointProjection::new(false, m * self.d());
            return (proj, TetrahedronPointLocation::OnVertex(3));
        }

        /*
         * Voronoï regions of edges.
         */
        #[inline(always)]
        fn check_edge<N: RealField>(
            i: usize,
            m: &Isometry<N>,
            a: &Point<N>,
            _: &Point<N>,
            nabc: &Vector<N>,
            nabd: &Vector<N>,
            ap: &Vector<N>,
            ab: &Vector<N>,
            ap_ab: N, /*ap_ac: N, ap_ad: N,*/
            bp_ab: N, /*bp_ac: N, bp_ad: N*/
        ) -> (
            N,
            N,
            Option<(PointProjection<N>, TetrahedronPointLocation<N>)>,
        )
        {
            let _0: N = na::zero();
            let _1: N = na::one();

            let ab_ab = ap_ab - bp_ab;

            // NOTE: The following avoids the subsequent cross and dot products but are not
            // numerically stable.
            //
            // let dabc  = ap_ab * (ap_ac - bp_ac) - ap_ac * ab_ab;
            // let dabd  = ap_ab * (ap_ad - bp_ad) - ap_ad * ab_ab;

            let ap_x_ab = ap.cross(ab);
            let dabc = ap_x_ab.dot(nabc);
            let dabd = ap_x_ab.dot(nabd);

            // FIXME: the case where ab_ab == _0 is not well defined.
            if ab_ab != _0 && dabc >= _0 && dabd >= _0 && ap_ab >= _0 && ap_ab <= ab_ab {
                // Voronoi region of `ab`.
                let u = ap_ab / ab_ab;
                let bcoords = [_1 - u, u];
                let res = a + ab * u;
                let proj = PointProjection::new(false, m * res);
                (
                    dabc,
                    dabd,
                    Some((proj, TetrahedronPointLocation::OnEdge(i, bcoords))),
                )
            } else {
                (dabc, dabd, None)
            }
        }

        // Voronoï region of ab.
        //            let bp_ad = bp_bd + bp_ab;
        //            let bp_ac = bp_bc + bp_ab;
        let nabc = ab.cross(&ac);
        let nabd = ab.cross(&ad);
        let (dabc, dabd, res) = check_edge(
            0,
            m,
            self.a(),
            self.b(),
            &nabc,
            &nabd,
            &ap,
            &ab,
            ap_ab,
            /*ap_ac, ap_ad,*/ bp_ab, /*, bp_ac, bp_ad*/
        );
        if let Some(res) = res {
            return res;
        }

        // Voronoï region of ac.
        // Substitutions (wrt. ab):
        //   b -> c
        //   c -> d
        //   d -> b
        //            let cp_ab = cp_ac - cp_bc;
        //            let cp_ad = cp_cd + cp_ac;
        let nacd = ac.cross(&ad);
        let (dacd, dacb, res) = check_edge(
            1,
            m,
            self.a(),
            self.c(),
            &nacd,
            &-nabc,
            &ap,
            &ac,
            ap_ac,
            /*ap_ad, ap_ab,*/ cp_ac, /*, cp_ad, cp_ab*/
        );
        if let Some(res) = res {
            return res;
        }

        // Voronoï region of ad.
        // Substitutions (wrt. ab):
        //   b -> d
        //   c -> b
        //   d -> c
        //            let dp_ac = dp_ad - dp_cd;
        //            let dp_ab = dp_ad - dp_bd;
        let (dadb, dadc, res) = check_edge(
            2,
            m,
            self.a(),
            self.d(),
            &-nabd,
            &-nacd,
            &ap,
            &ad,
            ap_ad,
            /*ap_ab, ap_ac,*/ dp_ad, /*, dp_ab, dp_ac*/
        );
        if let Some(res) = res {
            return res;
        }

        // Voronoï region of bc.
        // Substitutions (wrt. ab):
        //   a -> b
        //   b -> c
        //   c -> a
        //            let cp_bd = cp_cd + cp_bc;
        let nbcd = bc.cross(&bd);
        // NOTE: nabc = nbcd
        let (dbca, dbcd, res) = check_edge(
            3,
            m,
            self.b(),
            self.c(),
            &nabc,
            &nbcd,
            &bp,
            &bc,
            bp_bc,
            /*-bp_ab, bp_bd,*/ cp_bc, /*, -cp_ab, cp_bd*/
        );
        if let Some(res) = res {
            return res;
        }

        // Voronoï region of bd.
        // Substitutions (wrt. ab):
        //   a -> b
        //   b -> d
        //   d -> a

        //            let dp_bc = dp_bd - dp_cd;
        // NOTE: nbdc = -nbcd
        // NOTE: nbda = nabd
        let (dbdc, dbda, res) = check_edge(
            4,
            m,
            self.b(),
            self.d(),
            &-nbcd,
            &nabd,
            &bp,
            &bd,
            bp_bd,
            /*bp_bc, -bp_ab,*/ dp_bd, /*, dp_bc, -dp_ab*/
        );
        if let Some(res) = res {
            return res;
        }

        // Voronoï region of cd.
        // Substitutions (wrt. ab):
        //   a -> c
        //   b -> d
        //   c -> a
        //   d -> b
        // NOTE: ncda = nacd
        // NOTE: ncdb = nbcd
        let (dcda, dcdb, res) = check_edge(
            5,
            m,
            self.c(),
            self.d(),
            &nacd,
            &nbcd,
            &cp,
            &cd,
            cp_cd,
            /*-cp_ac, -cp_bc,*/ dp_cd, /*, -dp_ac, -dp_bc*/
        );
        if let Some(res) = res {
            return res;
        }

        /*
         * Voronoï regions of faces.
         */
        #[inline(always)]
        fn check_face<N: RealField>(
            i: usize,
            a: &Point<N>,
            b: &Point<N>,
            c: &Point<N>,
            m: &Isometry<N>,
            ap: &Vector<N>,
            bp: &Vector<N>,
            cp: &Vector<N>,
            ab: &Vector<N>,
            ac: &Vector<N>,
            ad: &Vector<N>,
            dabc: N,
            dbca: N,
            dacb: N,
            /* ap_ab: N, bp_ab: N, cp_ab: N,
            ap_ac: N, bp_ac: N, cp_ac: N, */
        ) -> Option<(PointProjection<N>, TetrahedronPointLocation<N>)>
        {
            let _0: N = na::zero();
            let _1: N = na::one();

            if dabc < _0 && dbca < _0 && dacb < _0 {
                let n = ab.cross(ac); // FIXME: is is possible to avoid this cross product?
                if n.dot(ad) * n.dot(ap) < _0 {
                    // Voronoï region of the face.

                    // NOTE:
                    // The following avoids expansive computations but are not very
                    // numerically stable.
                    //
                    // let va = bp_ab * cp_ac - cp_ab * bp_ac;
                    // let vb = cp_ab * ap_ac - ap_ab * cp_ac;
                    // let vc = ap_ab * bp_ac - bp_ab * ap_ac;

                    let normal = n.normalize();
                    let vc = normal.dot(&ap.cross(bp));
                    let va = normal.dot(&bp.cross(cp));
                    let vb = normal.dot(&cp.cross(ap));

                    let denom = va + vb + vc;
                    assert!(denom != _0);
                    let inv_denom = _1 / denom;

                    let bcoords = [va * inv_denom, vb * inv_denom, vc * inv_denom];
                    let res = a * bcoords[0] + b.coords * bcoords[1] + c.coords * bcoords[2];
                    let proj = PointProjection::new(false, m * res);

                    return Some((proj, TetrahedronPointLocation::OnFace(i, bcoords)));
                }
            }
            return None;
        }

        // Face abc.
        if let Some(res) = check_face(
            0,
            self.a(),
            self.b(),
            self.c(),
            m,
            &ap,
            &bp,
            &cp,
            &ab,
            &ac,
            &ad,
            dabc,
            dbca,
            dacb,
            /*ap_ab, bp_ab, cp_ab,
            ap_ac, bp_ac, cp_ac*/
        ) {
            return res;
        }

        // Face abd.
        if let Some(res) = check_face(
            1,
            self.a(),
            self.b(),
            self.d(),
            m,
            &ap,
            &bp,
            &dp,
            &ab,
            &ad,
            &ac,
            dadb,
            dabd,
            dbda,
            /*ap_ab, bp_ab, dp_ab,
            ap_ad, bp_ad, dp_ad*/
        ) {
            return res;
        }
        // Face acd.
        if let Some(res) = check_face(
            2,
            self.a(),
            self.c(),
            self.d(),
            m,
            &ap,
            &cp,
            &dp,
            &ac,
            &ad,
            &ab,
            dacd,
            dcda,
            dadc,
            /*ap_ac, cp_ac, dp_ac,
            ap_ad, cp_ad, dp_ad*/
        ) {
            return res;
        }
        // Face bcd.
        if let Some(res) = check_face(
            3,
            self.b(),
            self.c(),
            self.d(),
            m,
            &bp,
            &cp,
            &dp,
            &bc,
            &bd,
            &-ab,
            dbcd,
            dcdb,
            dbdc,
            /*bp_bc, cp_bc, dp_bc,
            bp_bd, cp_bd, dp_bd*/
        ) {
            return res;
        }

        if !solid {
            // XXX: implement the non-solid projection.
            unimplemented!("Non-solid ray-cast on a tetrahedron is not yet implemented.")
        }

        let proj = PointProjection::new(true, m * p);
        return (proj, TetrahedronPointLocation::OnSolid);
    }
}
