# Task: Robot arm reach & manipulability explorer (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code, tables or assets.
Use textbook kinematics only (standard Denavit–Hartenberg or modified DH, geometric Jacobian,
Yoshikawa manipulability). Presets are generic, clearly labelled "typical" arms with editable
parameters (no vendor names, no copied datasheet tables).

## Product (SMALL scope — core + UI about 800-1200 lines of new Rust; renderer/math are reused)
A single-page web app (GitHub Pages, sub-path `/arm-reach/`): "Here is my 6- or 7-DoF arm (DH table +
joint limits) — where can it reach, where is it dexterous, and how close is this pose to a singularity?"

1. **Inputs**: a DH table editor with 6 or 7 rows (a, alpha, d, theta offset; DH convention toggle
   standard / modified), per-joint limits (min/max deg), per-joint current angle slider.
   Presets (editable): "6-DoF spherical-wrist industrial arm (typical)", "7-DoF redundant
   collaborative arm (typical)", "6-DoF humanoid arm, shoulder-elbow-wrist (typical)".
   Units: m and deg. Sample count (2k / 5k / 10k, default 5k), seeded RNG (fixed seed, reproducible).
2. **Kinematics model (core)**: forward kinematics (all joint frames + tool frame), geometric
   Jacobian 6×N, manipulability w = sqrt(det(J·Jᵀ)) (translational 3×N part and full 6×N part,
   selectable), condition number of the translational part (via eigenvalues of the 3×3 J_v·J_vᵀ
   using a small symmetric Jacobi eigen solver — no linear-algebra crates), closest joint-limit
   margin. Workspace sampling: uniform random joint vectors inside limits → tool positions with
   their manipulability. Report: max reach from base, reach envelope (min/max radius, min/max z),
   % of samples with w below a "near-singular" threshold (user field, default 5% of max w).
3. **Extras (small)**: (a) "reach check" — a user target point (x, y, z): nearest sampled point,
   distance, and its joint vector (click to load into sliders); (b) per-pose readout of singular
   values of J_v (σ1 ≥ σ2 ≥ σ3) with a ✅ / ⚠️ near-singular badge.
4. **3D view (WebGL2, game-grade — see below)**: the arm drawn as lit capsules/cylinders between
   joint frames with joint housings (cylinders aligned to each joint z-axis), the tool frame as RGB
   axes; the workspace point cloud as small billboards/cubes coloured by manipulability (blue low →
   yellow high, near-singular red); the manipulability ellipsoid at the tool (translucent volume from
   J_v·J_vᵀ eigenvectors/values, scaled for visibility) with bright edges; the reach-check target as
   a draggable sphere with a marker label.
5. **2D plot** (canvas 2D, own small module): histogram of manipulability over samples with the current
   pose's w as a vertical marker; and a small bar chart of joint-limit margins for the current pose.
6. Korean + English toggle (browser default). Mobile friendly. CSS prefers-color-scheme only.
Out of scope (do NOT build): inverse kinematics solver, collision checking, dynamics, URDF import,
URL share, JSON import/export.

## 3D view requirements (game-like, reuse the existing renderer)
- `crates/ar_web/src/gl.rs` and `crates/ar_web/src/math.rs` already exist: a tool-independent WebGL2
  renderer (lit meshes, checker/grid ground with fog, sky gradient, stencil planar shadows, translucent
  volumes, lines) and pure math (mat4, damped orbit camera with inertia, ray-plane/ray-sphere picking)
  with unit tests. Reuse them; extend only if needed and keep them tool-independent and tested.
  For the point cloud, use one batched mesh/instance-like buffer (rebuilt only when samples change),
  not one draw call per point.
- Scene code in `view3d.rs`. Orbit (left drag) / pan (right drag, two fingers) / zoom (wheel, pinch)
  with inertia, double-click reset, `touch-action:none`. View presets: Perspective / Top / Side / Front.
- Direct manipulation: drag the reach-check **target sphere** on a horizontal plane (shift / second
  finger for height) and the nearest-sample readout updates live; hover highlight + cursor change.
  Clicking a joint housing selects that joint, and horizontal drag then changes its angle (clamped to
  limits); the pose, ellipsoid and readouts update live.
- Toggle buttons over the view: point cloud, ellipsoid, frames, near-singular only.
- Animation: "sweep" play/pause that moves the selected joint back and forth across its limits
  (speed 0.25× / 1×), so the ellipsoid and w visibly change. HUD over the canvas: tool x/y/z, w,
  σ-min, condition number, nearest limit margin.
- If WebGL2 is unavailable, show a message and keep the numeric readouts and 2D plots working.
- Release wasm ≲ 350 KB. No crates besides wasm-bindgen, web-sys, js-sys.

## Core crate (pure Rust, no web deps, unit tested) — `crates/ar_core`
- DH FK (both conventions), Jacobian, manipulability, Jacobi eigen 3×3, sampler with small seeded PRNG
  (e.g. xorshift/splitmix), reach stats, nearest-sample query.
- Tests (hand-checkable, 6-10): planar 2-link (a1=1, a2=1, all alpha=0) at q=(0,0) → tool (2,0,0) and
  at q=(0,90°) → (1,1,0); planar 2-link manipulability = a1·a2·|sin q2| (check q2=90° → 1, q2=0 → 0);
  Jacobian columns match finite differences of FK within 1e-6 for a random 6-DoF pose; standard vs
  modified DH give the same tool position for a converted 2-link example; Jacobi eigen of a known
  symmetric 3×3 (e.g. diag + rotation) recovers eigenvalues within 1e-9; sampler respects joint
  limits and is reproducible for a fixed seed; max reach of a straight 6-DoF preset ≤ sum of link
  lengths; nearest-sample query returns the exact sample when the target equals a sample.
- `crates/ar_web`: UI, plain web-sys. The existing math.rs tests must keep passing.

## Repo hygiene
- README (Korean first, then English): what it is, model and assumptions (rigid links, ideal joints,
  sampled — not exact — workspace, no collisions), controls, local build (`cargo test`,
  `trunk build --release`), MIT license, "educational estimates — validate before using on a real robot".
- LICENSE: MIT, copyright "arm-reach contributors". `.gitignore` for target/ dist/.
- `index.html` with a CSP meta that blocks external requests:
  `default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; base-uri 'self'`.
  In `Trunk.toml` set `filehash = false`, `public_url = "/arm-reach/"` and
  `pattern_script = "<script type=\"module\" src=\"{base}boot.js\"></script>"` with an external
  `web/boot.js` (copied via `<link data-trunk rel="copy-file">`) that imports `./ar_web.js` and calls init.
- `.github/workflows/pages.yml`: on push to main + workflow_dispatch; checkout, rust stable + wasm32
  target, install trunk 0.22.1 binary release, `sh tools/privacy_check.sh`, `cargo test --release`,
  `trunk build --release`, configure-pages@v5, upload-pages-artifact@v3 (dist), deploy job with
  actions/deploy-pages@v4 (permissions pages: write, id-token: write).
- `tools/privacy_check.sh` already exists; keep it and keep it clean.

## Done when
- `cargo test` green (workspace); `trunk build --release` succeeds with no warnings;
  `sh tools/privacy_check.sh` clean.
- Commit author `arm-reach contributors <59640857+roscore@users.noreply.github.com>`.
- Commit everything with clear commits; open a PR. Do NOT create new GitHub repos or change repo settings.
