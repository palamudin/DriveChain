#!/usr/bin/env python3
"""foam_case.py - generate a coarse OpenFOAM (v1912, simpleFoam, k-omega SST) external-aero case for a car mesh.

The case is written to a scratch directory OUTSIDE the repo (the car geometry is a private asset: never commit
the STL or the case).  Purpose of the coarse run: (1) wall-time feasibility on the box, (2) first check of the
acceptance criterion: does RANS on the bare geometry produce net lift with no sign or coefficient imposed?

Set-up (car frame, x fwd, y left, z up; air moves -x at U relative to the car):
  inlet x=+8 m, outlet x=-16 m, sides y=+-5 m, top z=ground+5 m (blockage ~ A/(10*5) = 3 %),
  ground = moving wall at the road speed (car frame), tyres = stationary walls (no wheel rotation in v0),
  ground plane lifted 10 mm into the tyres (standard contact-patch treatment), no prism layers (wall functions).
usage: foam_case.py <car.obj> <case_dir> [--offset=..] [--U 30] [--yaw 0] [--level 3]
"""
import os, sys, re, argparse
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from objload import load, wheel_x

ap = argparse.ArgumentParser()
ap.add_argument("obj"); ap.add_argument("case"); ap.add_argument("--offset", default="0,0,0")
ap.add_argument("--U", type=float, default=30.0); ap.add_argument("--yaw", type=float, default=0.0)
ap.add_argument("--level", type=int, default=3); ap.add_argument("--procs", type=int, default=4)
ap.add_argument("--iters", type=int, default=1200)
a = ap.parse_args()
assert not os.path.abspath(a.case).startswith(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))), \
    "case dir must be outside the repo (private geometry)"
V, F, G = load(a.obj, [float(t) for t in a.offset.split(",")])
xf, xr, zg = wheel_x(V, F, G); wb = xf - xr; xm = 0.5 * (xf + xr)
z0 = zg + 0.010
os.makedirs(f"{a.case}/constant/triSurface", exist_ok=True); os.makedirs(f"{a.case}/system", exist_ok=True)
os.makedirs(f"{a.case}/0", exist_ok=True)
wheel = np.array([bool(re.match(r"(tyre|tire|rim|brake|\w*disc|\w*hub)", g, re.I)) for g in G])   # wheel/brake groups
with open(f"{a.case}/constant/triSurface/car.stl", "w") as f:
    for name, m in (("body", ~wheel), ("wheels", wheel)):
        f.write(f"solid {name}\n")
        for t in V[F[m]]:
            n = np.cross(t[1] - t[0], t[2] - t[0]); ln = np.linalg.norm(n)
            if ln < 1e-14: continue
            n /= ln
            f.write(f"facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}\n outer loop\n")
            for p in t: f.write(f"  vertex {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
            f.write(" endloop\nendfacet\n")
        f.write(f"endsolid {name}\n")
H = lambda cls, obj: f'FoamFile {{ version 2.0; format ascii; class {cls}; object {obj}; }}\n'
b = np.radians(a.yaw); Uv = (-a.U * np.cos(b), -a.U * np.sin(b), 0.0)
Uroad = (-a.U * np.cos(b), 0.0, 0.0)   # ground moves at road speed along -x (car frame); crosswind is air-only
Ustr = "(%g %g %g)" % Uv
k_in = 1.5 * (0.005 * a.U) ** 2; om_in = k_in ** 0.5 / (0.09 ** 0.25 * 0.1)
x0, x1, y0, y1, z1, cell = -16.0, 8.0, -5.0, 5.0, z0 + 5.0, 0.25
nx, ny, nz = int((x1 - x0) / cell), int((y1 - y0) / cell), int((z1 - z0) / cell)
w = lambda p, s: open(f"{a.case}/{p}", "w").write(s)
w("system/blockMeshDict", H("dictionary", "blockMeshDict") + f"""
convertToMeters 1;
vertices ( ({x0} {y0} {z0}) ({x1} {y0} {z0}) ({x1} {y1} {z0}) ({x0} {y1} {z0})
           ({x0} {y0} {z1}) ({x1} {y0} {z1}) ({x1} {y1} {z1}) ({x0} {y1} {z1}) );
blocks ( hex (0 1 2 3 4 5 6 7) ({nx} {ny} {nz}) simpleGrading (1 1 1) );
boundary (
  outlet {{ type patch; faces ((0 4 7 3)); }}
  inlet  {{ type patch; faces ((1 2 6 5)); }}
  ground {{ type wall;  faces ((0 3 2 1)); }}
  top    {{ type patch; faces ((4 5 6 7)); }}
  sideR  {{ type patch; faces ((0 1 5 4)); }}
  sideL  {{ type patch; faces ((3 7 6 2)); }}
);
""")
L = a.level
w("system/snappyHexMeshDict", H("dictionary", "snappyHexMeshDict") + f"""
castellatedMesh true; snap true; addLayers false;
geometry {{
  car.stl {{ type triSurfaceMesh; name car; regions {{ body {{ name body; }} wheels {{ name wheels; }} }} }}
  near {{ type searchableBox; min ({xr-1.4} -1.1 {z0}); max ({xf+1.2} 1.1 {zg+1.45}); }}
  wake {{ type searchableBox; min ({xr-6.0} -1.6 {z0}); max ({xf+2.5} 1.6 {zg+2.0}); }}
}};
castellatedMeshControls {{
  maxLocalCells 4000000; maxGlobalCells 12000000; minRefinementCells 10; maxLoadUnbalance 0.10;
  nCellsBetweenLevels 3; features ();
  refinementSurfaces {{ car {{ level ({L} {L+1}); regions {{ body {{ level ({L} {L+1}); patchInfo {{ type wall; }} }}
                                                   wheels {{ level ({L} {L+1}); patchInfo {{ type wall; }} }} }} }} }}
  resolveFeatureAngle 30;
  refinementRegions {{ near {{ mode inside; levels ((1E15 {L})); }} wake {{ mode inside; levels ((1E15 {L-1})); }} }}
  locationInMesh ({x1-1.0} {y1-1.0} {z1-1.0});
  allowFreeStandingZoneFaces true;
}};
snapControls {{ nSmoothPatch 3; tolerance 2.0; nSolveIter 50; nRelaxIter 5; nFeatureSnapIter 5;
  implicitFeatureSnap true; explicitFeatureSnap false; multiRegionFeatureSnap false; }};
addLayersControls {{ relativeSizes true; layers {{ }}; expansionRatio 1.2; finalLayerThickness 0.5; minThickness 0.1;
  nGrow 0; featureAngle 60; nRelaxIter 3; nSmoothSurfaceNormals 1; nSmoothNormals 3; nSmoothThickness 10;
  maxFaceThicknessRatio 0.5; maxThicknessToMedialRatio 0.3; minMedialAxisAngle 90; nBufferCellsNoExtrude 0;
  nLayerIter 50; }};
meshQualityControls {{ #include "meshQualityDict" }};
mergeTolerance 1e-6;
""")
w("system/meshQualityDict", H("dictionary", "meshQualityDict") + """
maxNonOrtho 65; maxBoundarySkewness 20; maxInternalSkewness 4; maxConcave 80; minVol 1e-13; minTetQuality 1e-15;
minArea -1; minTwist 0.02; minDeterminant 0.001; minFaceWeight 0.05; minVolRatio 0.01; minTriangleTwist -1;
nSmoothScale 4; errorReduction 0.75;
""")
w("system/decomposeParDict", H("dictionary", "decomposeParDict") + f"numberOfSubdomains {a.procs}; method hierarchical; coeffs {{ n ({a.procs} 1 1); order xyz; }}\n")
w("system/controlDict", H("dictionary", "controlDict") + f"""
application simpleFoam; startFrom latestTime; startTime 0; stopAt endTime; endTime {a.iters}; deltaT 1;
writeControl timeStep; writeInterval {a.iters}; purgeWrite 1; writeFormat ascii; writePrecision 8;
writeCompression off; timeFormat general; timePrecision 6; runTimeModifiable true;
// no function objects: the Debian openfoam 1912 build aborts in functionObjectList with
// "error in IOstream sha1" (a known toolchain issue); forces are integrated offline by foam_forces.py
""")
w("system/fvSchemes", H("dictionary", "fvSchemes") + """
ddtSchemes { default steadyState; }
gradSchemes { default Gauss linear; grad(U) cellLimited Gauss linear 1; grad(k) cellLimited Gauss linear 1; grad(omega) cellLimited Gauss linear 1; }
divSchemes { default none; div(phi,U) bounded Gauss linearUpwind grad(U); div(phi,k) bounded Gauss upwind;
  div(phi,omega) bounded Gauss upwind; div((nuEff*dev2(T(grad(U))))) Gauss linear; }
laplacianSchemes { default Gauss linear limited corrected 0.33; }
interpolationSchemes { default linear; }
snGradSchemes { default limited corrected 0.33; }
wallDist { method meshWave; }
""")
w("system/fvSolution", H("dictionary", "fvSolution") + """
solvers {
  p { solver GAMG; smoother GaussSeidel; tolerance 1e-7; relTol 0.1; }
  "(U|k|omega)" { solver smoothSolver; smoother GaussSeidel; nSweeps 2; tolerance 1e-8; relTol 0.1; }
}
SIMPLE { nNonOrthogonalCorrectors 1; consistent yes; }
potentialFlow { nNonOrthogonalCorrectors 5; }
relaxationFactors { equations { U 0.7; k 0.5; omega 0.5; } fields { p 0.3; } }
""")
w("constant/transportProperties", H("dictionary", "transportProperties") + "transportModel Newtonian; nu 1.5e-05;\n")
w("constant/turbulenceProperties", H("dictionary", "turbulenceProperties") + "simulationType RAS; RAS { RASModel kOmegaSST; turbulence on; printCoeffs on; }\n")
walls_far = "top { type slip; } sideR { type slip; } sideL { type slip; }" if a.yaw == 0 else \
            f"top {{ type slip; }} sideR {{ type inletOutlet; inletValue uniform {Ustr}; value uniform {Ustr}; }} sideL {{ type inletOutlet; inletValue uniform {Ustr}; value uniform {Ustr}; }}"
w("0/U", H("volVectorField", "U") + f"""dimensions [0 1 -1 0 0 0 0]; internalField uniform {Ustr};
boundaryField {{ inlet {{ type fixedValue; value uniform {Ustr}; }} outlet {{ type inletOutlet; inletValue uniform (0 0 0); value uniform {Ustr}; }}
  ground {{ type movingWallVelocity; value uniform ({Uroad[0]} 0 0); }} {walls_far}
  "(body|wheels)" {{ type noSlip; }} "proc.*" {{ type processor; }} }}
""")
def scal(name, dim, val, wallbc):
    farbc = "slip" if a.yaw == 0 else "zeroGradient"
    return H("volScalarField", name) + f"""dimensions {dim}; internalField uniform {val};
boundaryField {{ inlet {{ type fixedValue; value uniform {val}; }} outlet {{ type inletOutlet; inletValue uniform {val}; value uniform {val}; }}
  top {{ type slip; }} sideR {{ type {farbc}; }} sideL {{ type {farbc}; }}
  "(ground|body|wheels)" {{ {wallbc} }} "proc.*" {{ type processor; }} }}
"""
w("0/k", scal("k", "[0 2 -2 0 0 0 0]", f"{k_in:.6g}", f"type kqRWallFunction; value uniform {k_in:.6g};"))
w("0/omega", scal("omega", "[0 0 -1 0 0 0 0]", f"{om_in:.6g}", f"type omegaWallFunction; value uniform {om_in:.6g};"))
w("0/nut", scal("nut", "[0 2 -1 0 0 0 0]", "0", "type nutkWallFunction; value uniform 0;"))
p0 = H("volScalarField", "p") + """dimensions [0 2 -2 0 0 0 0]; internalField uniform 0;
boundaryField { inlet { type zeroGradient; } outlet { type fixedValue; value uniform 0; }
  top { type slip; } sideR { type SIDE; } sideL { type SIDE; } "(ground|body|wheels)" { type zeroGradient; } "proc.*" { type processor; } }
"""
w("0/p", p0.replace("SIDE", "slip" if a.yaw == 0 else "zeroGradient"))
N = a.procs
w("Allrun", f"""#!/bin/bash
# coarse run; nice'd and on {N} cores so a concurrent perf benchmark on the box is not disturbed much
set -e; cd "$(dirname "$0")"; source /usr/share/openfoam/etc/bashrc 2>/dev/null || true
t0=$(date +%s)
if [ "$1" != "solve" ]; then
blockMesh > log.blockMesh 2>&1
decomposePar -force > log.decompose0 2>&1
nice -n 19 mpirun --oversubscribe -np {N} snappyHexMesh -overwrite -parallel > log.snappy 2>&1
t1=$(date +%s); echo "mesh_s $((t1-t0))" > timing.txt
fi; t1=$(date +%s)
nice -n 19 mpirun --oversubscribe -np {N} checkMesh -parallel > log.checkMesh 2>&1 || true
# copy initial fields into the decomposed mesh
for d in processor*; do rm -rf $d/0; cp -r 0 $d/0; done
nice -n 19 mpirun --oversubscribe -np {N} simpleFoam -parallel > log.simpleFoam 2>&1
t2=$(date +%s); echo "solve_s $((t2-t1)) iters {a.iters} procs {N}" >> timing.txt
""")
os.chmod(f"{a.case}/Allrun", 0o755)
print(f"case written to {a.case}: axles x {xf:+.3f}/{xr:+.3f}, wb {wb:.3f}, ground {zg:+.3f}, background {nx}x{ny}x{nz}, "
      f"surface level {L} ({cell/2**L*1000:.0f} mm), U {Ustr}")
