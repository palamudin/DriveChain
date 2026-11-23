# Native build on Windows (MSVC). Needs: Rust (rustup, stable-x86_64-pc-windows-msvc), VS 2022 Build Tools (C++),
# Vulkan SDK (for glslangValidator; VULKAN_SDK env var). Optional: -Dlss with DLSS_SDK pointing at the NVIDIA DLSS SDK.
param([switch]$Dlss, [string]$TarmacSrc = "")
$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")
if ($TarmacSrc) { $env:TARMAC_SRC = $TarmacSrc }
if (-not $env:VULKAN_SDK) { Write-Warning "VULKAN_SDK not set; glslangValidator must be on PATH" }
$features = @(); if ($Dlss) { $features = @("--features", "dlss") }
cargo build --release @features
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Host "-> target\release\drivechain-native.exe   (run: drivechain-native.exe  |  --still shot.png  |  --physics-bench 20)"
