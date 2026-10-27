//! Host check of a GXVISUAL pack against its world pack with the board's
//! own reader, validator and photo preparation (ASTC gate on), plus the
//! joint transforms at the gear-up / flaps-full / speedbrake-full extremes.
//!
//!   cargo run -p gx-render --example visual_check -- <pack.gxvisual> <world.gxworld>

use gx_render::photo::{joint_transforms, prepare_photo, transform_point, ASTC_EXTENSION};
use gx_world::{read_visual, read_world, Limits, VisualKind, VisualLimits};
use std::io::BufReader;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, pack, world] = args.as_slice() else {
        return Err("usage: visual_check <pack.gxvisual> <world.gxworld>".into());
    };
    let world = read_world(BufReader::new(std::fs::File::open(world).map_err(|e| e.to_string())?), Limits::default())
        .map_err(|e| format!("world: {e:?}"))?;
    let visual = read_visual(BufReader::with_capacity(1 << 20, std::fs::File::open(pack).map_err(|e| e.to_string())?), VisualLimits::default())?;
    let plan = prepare_photo(&world, &visual, ASTC_EXTENSION, VisualLimits::default())?;
    let aircraft: Vec<_> = plan.batches.iter().filter(|b| b.kind == VisualKind::Aircraft).collect();
    println!(
        "GXVISUAL {}.{}: {} textures, {} batches ({} aircraft), {} joints, triangles {:?}, gpu static {} bytes",
        visual.format_major, visual.format_minor, plan.textures.len(), plan.batches.len(), aircraft.len(),
        plan.joints.len(), plan.memory.triangles, plan.memory.gpu_static_bytes
    );
    // Lowest aircraft point (FRD +z is down) at gear down / gear up.
    let lowest = |drivers: [f32; 6]| {
        let m = joint_transforms(&plan.joints, &drivers);
        aircraft
            .iter()
            .flat_map(|b| &plan.indices[b.first_index as usize..(b.first_index + b.index_count) as usize])
            .map(|&i| {
                let v = plan.vertices[i as usize];
                transform_point(&m[v.animation as usize], v.position)[2]
            })
            .fold(f32::MIN, f32::max)
    };
    println!("lowest point FRD z: gear down {:.3} m, gear up {:.3} m", lowest([0.0, 0.0, 0.0, 0.0, 0.0, 1.0]), lowest([0.0; 6]));
    let full = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
    let m = joint_transforms(&plan.joints, &full);
    if m.iter().flatten().flatten().any(|v| !v.is_finite()) {
        return Err("non-finite joint transform".into());
    }
    Ok(())
}
