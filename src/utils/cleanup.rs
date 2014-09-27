use nalgebra::na::Vec3;

/// Given an index buffer, remove from `points` every point that is not indexed.
pub fn remove_unused_points<V>(points: &mut Vec<V>, idx: &mut [Vec3<u32>]) {
    let mut used  = Vec::from_elem(points.len(), false);
    let mut remap = Vec::from_fn(points.len(), |i| i);
    let used      = used.as_mut_slice();
    let remap     = remap.as_mut_slice();

    for i in idx.iter() {
        used[i.x as uint] = true;
        used[i.y as uint] = true;
        used[i.z as uint] = true;
    }

    let mut i = 0;
    while i != points.len() {
        if !used[i] {
            let _ = points.swap_remove(i);
            remap[points.len()] = i;
            used[i] = used[points.len()];
        }
        else {
            i = i + 1;
        }
    }

    for id in idx.iter_mut() {
        id.x = remap[id.x as uint] as u32;
        id.y = remap[id.y as uint] as u32;
        id.z = remap[id.z as uint] as u32;
    }
}
