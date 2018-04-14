use std::iter;
use na::Point3;

/// Given an index buffer, remove from `points` every point that is not indexed.
pub fn remove_unused_points<V>(points: &mut Vec<Vector<N>>, idx: &mut [Point3<u32>]) {
    let mut used: Vec<bool> = iter::repeat(false).take(points.len()).collect();
    let mut remap: Vec<usize> = (0..points.len()).map(|i| i).collect();
    let used = &mut used[..];
    let remap = &mut remap[..];

    for i in idx.iter() {
        used[i.x as usize] = true;
        used[i.y as usize] = true;
        used[i.z as usize] = true;
    }

    let mut i = 0;
    while i != points.len() {
        if !used[i] {
            let _ = points.swap_remove(i);
            remap[points.len()] = i;
            used[i] = used[points.len()];
        } else {
            i = i + 1;
        }
    }

    for id in idx.iter_mut() {
        id.x = remap[id.x as usize] as u32;
        id.y = remap[id.y as usize] as u32;
        id.z = remap[id.z as usize] as u32;
    }
}
