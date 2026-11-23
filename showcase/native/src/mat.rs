//! Material-group switches: render only what a test needs, and bisect artefacts by material group on a soft GPU
//! (lavapipe) or on the real rig. Every group can be on / flat (unlit, textured) / debug (unlit solid colour) / off (culled
//! from every pass, incl. shadows), plus feature overrides: opaque (no alpha test), nomips (mip 0 only), noshadow (casts no
//! shadow), nospec (no specular).
//!   CLI:  --mat leaves=off,grass=flat,impostors=debug+nomips,all=on      --isolate leaves[,bark]   (others off, GI/volumetrics/
//!         bloom skipped, cheap sky)        render.ini next to the exe: [materials] leaves = flat+opaque   isolate = leaves
//!   Keys: F3 next group, F4 next mode (on -> flat -> debug -> off), F6 pick feature override, F7 toggle it on the group, F9 reset all; F5 TAA, F8 spec-AA; shown on screen, logged.
pub const GROUPS: [&str; 11] = ["leaves", "bark", "impostors", "shrubs", "grass", "markings", "asphalt", "kerbs", "terrain", "props", "car"];
pub const MODES: [&str; 4] = ["on", "flat", "debug", "off"];
pub const FLAGS: [&str; 4] = ["opaque", "nomips", "noshadow", "nospec"];
pub const G_LEAVES: usize = 0; pub const G_BARK: usize = 1; pub const G_IMP: usize = 2; pub const G_SHRUBS: usize = 3; pub const G_GRASS: usize = 4;
pub const G_MARKINGS: usize = 5; pub const G_CAR: usize = 10;
pub const OFF: u8 = 3; pub const F_NOSHADOW: u8 = 4;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mats { pub mode: [u8; 11], pub flags: [u8; 11], pub isolate: bool, pub sel: usize, pub sel_flag: usize }
impl Default for Mats { fn default() -> Self { Mats { mode: [0; 11], flags: [0; 11], isolate: false, sel: 0, sel_flag: 0 } } }

/// World surface / visual class -> group (DriveChain surf ids 0..11, visual cls 10/32/33/34, car parts >= 100, 12 = paint decals)
pub fn world_group(surf: u32) -> usize {
    match surf { 0 | 8 => 6, 5 | 12 => 5, 1 => 7, 2 | 3 | 6 | 7 | 11 => 8, s if s >= 100 => 10, 32 => 0, _ => 9 }
}

impl Mats {
    /// "leaves=off,grass=flat+nomips,all=debug" (group or 'all' = mode[+flag...] or just +flag...)
    pub fn apply(&mut self, spec: &str) -> Result<(), String> {
        for item in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            let (g, v) = item.split_once('=').ok_or(format!("material switch '{item}': expected group=mode[+flag]"))?;
            let gs: Vec<usize> = if g.trim() == "all" { (0..GROUPS.len()).collect() } else { vec![GROUPS.iter().position(|n| *n == g.trim()).ok_or(format!("unknown material group '{}' (groups: {})", g.trim(), GROUPS.join(" ")))?] };
            for part in v.split('+').map(str::trim).filter(|s| !s.is_empty()) {
                if let Some(m) = MODES.iter().position(|n| *n == part) { for &k in &gs { self.mode[k] = m as u8; } }
                else if let Some(f) = FLAGS.iter().position(|n| *n == part) { for &k in &gs { self.flags[k] |= 1 << f; } }
                else if part == "clear" { for &k in &gs { self.flags[k] = 0; } }
                else { return Err(format!("unknown mode/flag '{part}' (modes: {}; flags: {})", MODES.join(" "), FLAGS.join(" "))); }
            }
        }
        Ok(())
    }
    pub fn isolate(&mut self, groups: &str) -> Result<(), String> {
        let keep: Vec<usize> = groups.split(',').map(str::trim).filter(|s| !s.is_empty()).map(|g| GROUPS.iter().position(|n| *n == g).ok_or(format!("unknown material group '{g}'"))).collect::<Result<_, _>>()?;
        for k in 0..GROUPS.len() { if !keep.contains(&k) { self.mode[k] = OFF; } } self.isolate = true; Ok(())
    }
    /// render.ini [materials] (optional, next to the exe), then --mat / --isolate (CLI wins).
    pub fn from_args(ini: Option<&std::path::Path>) -> Mats {
        let mut m = Mats::default(); let log = |e: String| crate::logln(&format!("materials: {e}"));
        if let Some(p) = ini.filter(|p| p.exists()) { if let Ok(t) = std::fs::read_to_string(p) { let mut sec = String::new();
            for l in t.lines() { let l = l.split(';').next().unwrap_or("").trim(); if l.starts_with('[') { sec = l.trim_matches(['[', ']']).trim().to_lowercase(); continue; }
                if sec != "materials" { continue; } if let Some((k, v)) = l.split_once('=') { let (k, v) = (k.trim(), v.trim()); if v.is_empty() { continue; }
                    let r = if k == "isolate" { m.isolate(v) } else { m.apply(&format!("{k}={v}")) }; if let Err(e) = r { log(e); } } } } }
        let a: Vec<String> = std::env::args().collect(); let get = |n: &str| a.iter().position(|x| x == n).and_then(|i| a.get(i + 1)).cloned();
        if let Some(s) = get("--mat") { if let Err(e) = m.apply(&s) { log(e); } }
        if let Some(s) = get("--isolate") { if let Err(e) = m.isolate(&s) { log(e); } }
        m
    }
    pub fn visible(&self, g: usize) -> bool { self.mode[g] != OFF }
    pub fn casts(&self, g: usize) -> bool { self.visible(g) && self.flags[g] & F_NOSHADOW == 0 }
    /// Shader word: bits 0-1 mode (0 on 1 flat 2 debug), bits 4-7 flags (opaque nomips noshadow nospec)
    pub fn words(&self) -> [[u32; 4]; 3] { let mut w = [[0u32; 4]; 3]; for g in 0..GROUPS.len() { w[g / 4][g % 4] = (self.mode[g] as u32 & 3) | ((self.flags[g] as u32) << 4); } w }
    pub fn is_default(&self) -> bool { self.mode == [0; 11] && self.flags == [0; 11] }
    pub fn describe(&self) -> String {
        if self.is_default() { return "all groups on".into(); }
        (0..GROUPS.len()).filter(|&g| self.mode[g] != 0 || self.flags[g] != 0).map(|g| { let mut s = format!("{}={}", GROUPS[g], MODES[self.mode[g] as usize]);
            for (f, n) in FLAGS.iter().enumerate() { if self.flags[g] & (1 << f) != 0 { s.push('+'); s.push_str(n); } } s }).collect::<Vec<_>>().join(",")
    }
    /// Overlay lines (F3/F4/F6 hotkeys); the selected group is marked.
    pub fn overlay_lines(&self) -> Vec<String> {
        let mut v = vec!["MATERIALS F3 group F4 mode F9 reset".to_string(), "F6 pick override, F7 toggle it".to_string()];
        let mut line = String::new();
        for g in 0..GROUPS.len() { let mut s = format!("{}{}:{}", if g == self.sel { ">" } else { " " }, GROUPS[g], MODES[self.mode[g] as usize]);
            for (f, n) in FLAGS.iter().enumerate() { if self.flags[g] & (1 << f) != 0 { s.push('+'); s.push_str(&n[..3]); } }
            if line.len() + s.len() > 42 { v.push(std::mem::take(&mut line)); } line.push_str(&s); line.push(' '); }
        v.push(line); v.push(format!(" F7: {} on {}", FLAGS[self.sel_flag], GROUPS[self.sel])); v
    }
    pub fn key_group(&mut self) { self.sel = (self.sel + 1) % GROUPS.len(); }
    pub fn key_mode(&mut self) { self.mode[self.sel] = (self.mode[self.sel] + 1) % 4; }
    pub fn key_flag_pick(&mut self) { self.sel_flag = (self.sel_flag + 1) % FLAGS.len(); }
    pub fn key_flag(&mut self) { self.flags[self.sel] ^= 1 << self.sel_flag; }
    pub fn reset(&mut self) { let (s, f) = (self.sel, self.sel_flag); *self = Mats::default(); self.sel = s; self.sel_flag = f; }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn parse() {
        let mut m = Mats::default(); m.apply("leaves=off,grass=flat+nomips,impostors=debug").unwrap();
        assert_eq!(m.mode[G_LEAVES], OFF); assert_eq!(m.mode[G_GRASS], 1); assert_eq!(m.flags[G_GRASS], 2); assert_eq!(m.mode[G_IMP], 2);
        assert_eq!(m.describe(), "leaves=off,impostors=debug,grass=flat+nomips"); assert!(m.apply("nope=on").is_err()); assert!(m.apply("leaves=sparkly").is_err());
        let mut m = Mats::default(); m.isolate("leaves,bark").unwrap(); assert!(m.visible(G_LEAVES) && m.visible(G_BARK) && !m.visible(G_GRASS) && !m.visible(G_CAR));
        let mut m = Mats::default(); m.apply("all=debug,car=on+noshadow").unwrap(); assert_eq!(m.mode[G_CAR], 0); assert!(!m.casts(G_CAR)); assert_eq!(m.words()[2][2], 64);
        assert_eq!(world_group(0), 6); assert_eq!(world_group(12), 5); assert_eq!(world_group(101), 10);
    }
}
