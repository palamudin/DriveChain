//! build.rs
//!  1. Compiles Jakov's Tarmac native core (NativeCurrent287, C++17) + our C wrapper (tms.cpp) with the cc crate, with the
//!     exact flags of the web/wasm build (tmcore/cc.sh). Upstream sources live in ../tmcore/upstream (PRIVATE, not in git);
//!     override with TARMAC_SRC=<dir containing WheelPreparation92/...>.
//!  2. Compiles every shader in shaders/ to SPIR-V with glslangValidator (Vulkan 1.3 / SPIR-V 1.6 target).
//!  3. Feature `dlss`: compiles native/dlss_shim.cpp against DLSS_SDK (NVIDIA NGX). Untested here (no SDK, no GPU).
use std::{env, path::PathBuf, process::Command};

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let tm = manifest.join("../tmcore");
    let up = env::var("TARMAC_SRC").map(PathBuf::from).unwrap_or_else(|_| tm.join("upstream"));
    let target = env::var("TARGET").unwrap();
    let native = up.join("WheelPreparation92/native");
    if !native.join("vehicle.cpp").exists() {
        panic!("Tarmac sources not found at {} (private, not in git). Set TARMAC_SRC.", up.display());
    }
    let mut b = cc::Build::new();
    b.cpp(true).std("c++17").opt_level(2).warnings(false)
        .flag_if_supported("-ffp-contract=off").flag_if_supported("-fno-fast-math")
        .define("TARMAC_FRONT_TYRE176", None).define("TARMAC_TRACTION188", None).define("TARMAC_TYRE_RESPONSE192", None)
        .include(&native).include(up.join("FrontTyreObserver176")).include(up.join("JoypadSpan186")).include(up.join("TyreResponse192"))
        .include(tm.join("shim")).include(tm.join("shim/rel/a/b")).include(tm.join("api"))
        .flag("-include").flag(tm.join("shim/port_prelude.h").to_str().unwrap());
    if target.contains("windows-gnu") {
        // mingw-w64 posix threads flavour (std::thread in the wheel workers)
        b.compiler("x86_64-w64-mingw32-g++-posix");
        b.flag("-static-libgcc").flag("-static-libstdc++");
    } else if target.contains("msvc") {
        b.flag("/EHsc").flag("/bigobj").flag("/fp:strict");
    }
    for f in ["city_surface.cpp", "vehicle.cpp", "envelope141.cpp", "wheel_workers92.cpp", "abi.cpp"] {
        b.file(native.join(f));
        println!("cargo:rerun-if-changed={}", native.join(f).display());
    }
    b.file(tm.join("api/tms.cpp"));
    println!("cargo:rerun-if-changed={}", tm.join("api/tms.cpp").display());
    b.compile("tarmac");
    if target.contains("windows-gnu") {
        println!("cargo:rustc-link-arg=-static");
        println!("cargo:rustc-link-lib=static=stdc++");
        println!("cargo:rustc-link-lib=static=winpthread");
    } else if !target.contains("msvc") {
        println!("cargo:rustc-link-lib=stdc++");
    }

    // shaders
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let glslang = env::var("VULKAN_SDK").ok().map(|s| PathBuf::from(s).join("Bin").join("glslangValidator.exe"))
        .filter(|p| p.exists()).unwrap_or_else(|| PathBuf::from("glslangValidator"));
    println!("cargo:rerun-if-changed=shaders");
    for e in std::fs::read_dir(manifest.join("shaders")).unwrap() {
        let p = e.unwrap().path();
        let ext = p.extension().and_then(|x| x.to_str()).unwrap_or("");
        if !matches!(ext, "vert" | "frag" | "comp") { continue; }
        let name = p.file_name().unwrap().to_str().unwrap().to_string();
        let mut variants = vec![(name.clone(), vec![])];
        if name == "shadow_rt.comp" || name == "gi_probe.comp" { variants.push((name.replace(".comp", "_rq.comp"), vec!["-DRAY_QUERY=1".to_string()])); }
        for (vn, defs) in variants {
            let o = out.join(format!("{vn}.spv"));
            let st = Command::new(&glslang).args(["-V", "--target-env", "vulkan1.3"]).arg(format!("-I{}", manifest.join("shaders").display()))
                .args(&defs).arg(&p).arg("-o").arg(&o).status().expect("glslangValidator not found (install glslang-tools or the Vulkan SDK)");
            assert!(st.success(), "shader compile failed: {}", p.display());
        }
    }

    if env::var("CARGO_FEATURE_DLSS").is_ok() {
        let sdk = PathBuf::from(env::var("DLSS_SDK").expect("feature dlss needs DLSS_SDK"));
        cc::Build::new().cpp(true).std("c++17").file(manifest.join("native/dlss_shim.cpp"))
            .include(sdk.join("include")).compile("dlss_shim");
        println!("cargo:rustc-link-search={}", sdk.join("lib/Windows_x86_64/x64").display());
        println!("cargo:rustc-link-lib=nvsdk_ngx_s");
    }
}
