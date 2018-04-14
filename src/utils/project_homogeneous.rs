use na::FromHomogeneous;
use na;

/// Project n-d point to a (n-1)-d space, dividing each vector by its `w` component.
pub fn project_homogeneous<V: FromHomogeneous<VHomo>, VHomo>(pts: &mut [VHomo]) -> Vec<Vector<N>> {
    let mut res = Vec::with_capacity(pts.len());

    project_homogeneous_to(pts, &mut res);

    res
}

/// Project n-d point to a (n-1)-d space, dividing each vector by its `w` component.
pub fn project_homogeneous_to<V: FromHomogeneous<VHomo>, VHomo>(pts: &[VHomo], out: &mut Vec<Vector<N>>) {
    for pt in pts.iter() {
        out.push(na::from_homogeneous(pt));
    }
}
