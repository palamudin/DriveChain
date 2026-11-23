//! Swappable glTF world loader. Same convention as the web build (web/js/world.js):
//!   node/ancestor extras.tarmac.collide => collision mesh with extras.tarmac.surf (DriveChain surface id),
//!   otherwise "COL_<surf>" node names; everything else is visual (extras.tarmac.cls = visual class).
//!   Scene extras.tarmac = { name, spawns:[{name,x,z,yaw}], line:[[x,y,z],...] }.
use anyhow::{Context, Result};
use glam::{Mat4, Vec3};

pub const SURF_NAMES: [&str; 12] = ["tarmac", "kerb", "grass", "gravel", "sausage", "line", "pave", "rough", "pad", "wall", "other", "concrete"];

#[derive(Default)]
pub struct MeshPart { pub name: String, pub collide: bool, pub surf: u8, pub cls: i32, pub positions: Vec<[f32; 3]>, pub normals: Vec<[f32; 3]>, pub uvs: Vec<[f32; 2]>, pub indices: Vec<u32> }
#[derive(Clone, Copy, Debug)]
pub struct Spawn { pub x: f64, pub z: f64, pub yaw: f64 }
pub struct World { pub name: String, pub parts: Vec<MeshPart>, pub spawns: Vec<Spawn>, pub line: Vec<Vec3> }

fn tarmac_extras(e: &Option<Box<gltf::json::extras::RawValue>>) -> Option<serde_json::Value> {
    e.as_ref().and_then(|r| serde_json::from_str::<serde_json::Value>(r.get()).ok()).and_then(|v| v.get("tarmac").cloned())
}

pub fn load(path: &str) -> Result<World> {
    let (doc, buffers, _images) = gltf::import(path).with_context(|| format!("loading {path}"))?;
    let scene = doc.default_scene().or_else(|| doc.scenes().next()).context("no scene")?;
    let meta = tarmac_extras(scene.extras()).unwrap_or(serde_json::json!({}));
    let mut parts = Vec::new();
    fn walk(n: gltf::Node, parent: Mat4, inherited: Option<serde_json::Value>, buffers: &[gltf::buffer::Data], parts: &mut Vec<MeshPart>) {
        let m = parent * Mat4::from_cols_array_2d(&n.transform().matrix());
        let t = tarmac_extras(n.extras()).or(inherited);
        if let Some(mesh) = n.mesh() {
            let name = n.name().unwrap_or("").to_string();
            let collide = t.as_ref().and_then(|v| v.get("collide")).and_then(|v| v.as_bool()).unwrap_or(name.starts_with("COL_"));
            let surf = t.as_ref().and_then(|v| v.get("surf")).and_then(|v| v.as_u64()).map(|v| v as u8)
                .unwrap_or_else(|| SURF_NAMES.iter().position(|s| name.trim_start_matches("COL_") == *s).unwrap_or(0) as u8);
            let cls = t.as_ref().and_then(|v| v.get("cls")).and_then(|v| v.as_i64()).unwrap_or(-1) as i32;
            for prim in mesh.primitives() {
                let r = prim.reader(|b| Some(&buffers[b.index()]));
                let Some(pos) = r.read_positions() else { continue };
                let positions: Vec<[f32; 3]> = pos.map(|p| m.transform_point3(Vec3::from(p)).to_array()).collect();
                let normals: Vec<[f32; 3]> = r.read_normals().map(|it| it.map(|n| m.transform_vector3(Vec3::from(n)).normalize_or_zero().to_array()).collect()).unwrap_or_default();
                let uvs: Vec<[f32; 2]> = r.read_tex_coords(0).map(|t| t.into_f32().collect()).unwrap_or_default();
                let indices: Vec<u32> = r.read_indices().map(|i| i.into_u32().collect()).unwrap_or_else(|| (0..positions.len() as u32).collect());
                parts.push(MeshPart { name: name.clone(), collide, surf, cls, positions, normals, uvs, indices });
            }
        }
        for c in n.children() { walk(c, m, t.clone(), buffers, parts); }
    }
    for n in scene.nodes() { walk(n, Mat4::IDENTITY, None, &buffers, &mut parts); }
    let spawns = meta.get("spawns").and_then(|s| s.as_array()).map(|a| a.iter().map(|s| Spawn {
        x: s["x"].as_f64().unwrap_or(0.0), z: s["z"].as_f64().unwrap_or(0.0), yaw: s["yaw"].as_f64().unwrap_or(0.0) }).collect()).unwrap_or_else(|| vec![Spawn { x: 0.0, z: 0.0, yaw: 0.0 }]);
    let line = meta.get("line").and_then(|s| s.as_array()).map(|a| a.iter().filter_map(|p| { let p = p.as_array()?; Some(Vec3::new(p[0].as_f64()? as f32, p[1].as_f64()? as f32, p[2].as_f64()? as f32)) }).collect()).unwrap_or_default();
    Ok(World { name: meta.get("name").and_then(|v| v.as_str()).unwrap_or(path).to_string(), parts, spawns, line })
}

impl World {
    /// Collision soup for tms_world_create.
    pub fn collision(&self) -> (Vec<f32>, Vec<u32>, Vec<u8>) {
        let (mut p, mut i, mut s) = (Vec::new(), Vec::new(), Vec::new());
        for part in self.parts.iter().filter(|p| p.collide) {
            let base = (p.len() / 3) as u32;
            for v in &part.positions { p.extend_from_slice(v); }
            for t in part.indices.chunks(3) { if t.len() == 3 { i.extend(t.iter().map(|x| x + base)); s.push(part.surf); } }
        }
        (p, i, s)
    }
}
