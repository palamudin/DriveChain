# TARMAC reference sources

`tyre.js` (tyre carcass and contact geometry) and `physics.js` (vehicle physics, of which the tyre-force part is
ported) are excerpts of **TARMAC Vehicle Dynamics Lab 0.5.0**, written by the project owner and contributed to this
repository as the reference for the C port in `core/src/dc_tyre_tm.c` (`tyre_model = 1`).

They are kept verbatim for review and regression of the port; they are not built or shipped. Deviations of the port
from these sources are listed in `docs/TYRE_TARMAC.md`.
