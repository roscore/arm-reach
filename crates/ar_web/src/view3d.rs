//! 3D scene: arm, workspace cloud, manipulability ellipsoid, reach target; camera and picking.
//! The robot is z-up; the renderer is y-up with the ground at y = 0. `to_view` / `from_view`
//! are the only place the axes are swapped.
use crate::gl::{self, Mesh, Paint, Renderer};
use crate::math::{self, Orbit, OrbitCam, M4, V};
use ar_core::{Arm, Convention, Pose, M3, V3};
use web_sys::HtmlCanvasElement;

pub fn to_view(p: V3) -> V {
    [p[0] as f32, p[2] as f32, -p[1] as f32]
}
pub fn from_view(v: V) -> V3 {
    [v[0] as f64, -v[2] as f64, v[1] as f64]
}

/// Everything the scene needs from the app for one frame.
pub struct Scene<'a> {
    pub arm: &'a Arm,
    pub pose: &'a Pose,
    pub sigma: V3,
    pub dirs: M3,
    pub target: V3,
    pub nearest: Option<V3>,
    pub sel: Option<usize>,
    /// point cloud, ellipsoid, frames, near-singular only
    pub show: [bool; 4],
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Hit {
    None,
    Target,
    Joint(usize),
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Drag {
    None,
    Orbit,
    Pan,
    Target,
    Joint(usize),
}

pub struct View {
    pub r: Renderer,
    pub cam: OrbitCam,
    pub canvas: HtmlCanvasElement,
    /// Characteristic arm size (m); sets radii, camera distance and fog.
    pub size: f32,
    center: V,
    cyl: Mesh,
    sph: Mesh,
    axes: Mesh,
    rings: Mesh,
    cloud: Option<Mesh>,
    cloud_n: [i32; 2],
    inv_vp: M4,
    vp: M4,
    px: [f32; 2],
    pub drag: Drag,
    pub hover: Hit,
    /// Active pointers: (id, x, y) in CSS px.
    pub ptrs: Vec<(i32, f32, f32)>,
}

fn rgb_lines(pts: &[V], cols: &[V]) -> gl::Geo {
    let mut g = gl::lines(pts);
    g.col = cols.iter().flatten().copied().collect();
    g
}

impl View {
    pub fn new(canvas: HtmlCanvasElement, dark: bool) -> Option<View> {
        let mut r = Renderer::new(&canvas)?;
        r.theme = if dark { gl::DARK } else { gl::LIGHT };
        let (x, y, z) = ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
        let axes = rgb_lines(&[[0.0; 3], x, [0.0; 3], y, [0.0; 3], z], &[x, x, y, y, z, z]);
        // Three great circles of the unit sphere, for the ellipsoid edges.
        let circ: Vec<V> = (0..=48).map(|i| (i as f32 / 48.0 * std::f32::consts::TAU).sin_cos()).map(|(s, c)| [c, s, 0.0]).collect();
        let mut ring = vec![];
        for p in circ.windows(2) {
            for m in [|v: V| v, |v: V| [v[0], 0.0, v[1]], |v: V| [0.0, v[0], v[1]]] {
                ring.extend([m(p[0]), m(p[1])]);
            }
        }
        Some(View {
            cyl: r.mesh(&gl::cylinder(20))?,
            sph: r.mesh(&gl::sphere(16))?,
            axes: r.mesh(&axes)?,
            rings: r.mesh(&gl::lines(&ring))?,
            r,
            cam: OrbitCam::new(Orbit { target: [0.0; 3], yaw: 0.7, pitch: 0.45, dist: 4.0 }),
            canvas,
            size: 1.0,
            center: [0.0; 3],
            cloud: None,
            cloud_n: [0; 2],
            inv_vp: math::IDENTITY,
            vp: math::IDENTITY,
            px: [1.0; 2],
            drag: Drag::None,
            hover: Hit::None,
            ptrs: vec![],
        })
    }

    /// Fit the camera and ground to an arm of size `size` around `center` (robot frame).
    pub fn fit(&mut self, size: f64, center: V3) {
        self.size = size.max(0.1) as f32;
        self.center = to_view(center);
        self.r.fog = [self.size * 5.0, self.size * 14.0];
        self.r.cell = [0.05, 0.1, 0.25, 0.5, 1.0].into_iter().find(|c| *c * 6.0 >= self.size).unwrap_or(1.0);
        self.preset(0);
    }

    /// 0 perspective, 1 top, 2 side (looking along +y robot), 3 front (looking along −x robot).
    pub fn preset(&mut self, k: usize) {
        let (yaw, pitch) = [(0.7, 0.45), (0.0, math::PITCH_MAX), (0.0, 0.0), (std::f32::consts::FRAC_PI_2, 0.0)][k.min(3)];
        self.cam.set_goal(Orbit { target: self.center, yaw, pitch, dist: self.size * 2.4 });
    }

    /// Rebuild the batched cloud. Points must be ordered near-singular first; `near` counts them.
    pub fn set_cloud(&mut self, pts: &[V3], cols: &[[f32; 3]], near: usize) {
        if let Some(m) = self.cloud.take() {
            self.r.free(m);
        }
        let p: Vec<V> = pts.iter().map(|&p| to_view(p)).collect();
        self.cloud = self.r.mesh(&gl::points(&p, cols, self.size * 0.0045));
        self.cloud_n = [p.len() as i32 * 24, near as i32 * 24];
    }

    fn ndc(&self, x: f32, y: f32) -> (V, V) {
        math::ndc_ray(&self.inv_vp, 2.0 * x / self.px[0] - 1.0, 1.0 - 2.0 * y / self.px[1])
    }

    /// Project a robot-frame point to CSS px, if in front of the camera.
    pub fn project(&self, p: V3) -> Option<(f32, f32)> {
        let c = math::transform(&self.vp, to_view(p));
        (c[2].abs() < 1.0).then(|| ((c[0] + 1.0) * 0.5 * self.px[0], (1.0 - c[1]) * 0.5 * self.px[1]))
    }

    fn housing_r(&self) -> f32 {
        self.size * 0.032
    }

    pub fn pick(&self, x: f32, y: f32, sc: &Scene) -> Hit {
        let (o, d) = self.ndc(x, y);
        let mut best = (f32::INFINITY, Hit::None);
        if let Some(t) = math::ray_sphere(o, d, to_view(sc.target), self.size * 0.04) {
            best = (t, Hit::Target);
        }
        for (i, &(c, _)) in sc.pose.axes.iter().enumerate() {
            if let Some(t) = math::ray_sphere(o, d, to_view(c), self.housing_r() * 1.4) {
                if t < best.0 {
                    best = (t, Hit::Joint(i));
                }
            }
        }
        best.1
    }

    /// New target position under the pointer: on the horizontal plane through the target, or
    /// (vertical) on the upright plane facing the camera, changing only the height.
    pub fn drag_target(&self, x: f32, y: f32, target: V3, vertical: bool) -> Option<V3> {
        let (o, d) = self.ndc(x, y);
        let t = to_view(target);
        if vertical {
            let e = self.cam.cur.eye();
            let n = math::norm([e[0] - t[0], 0.0, e[2] - t[2]]);
            let h = math::ray_plane(o, d, t, n)?;
            Some(from_view([t[0], h[1], t[2]]))
        } else {
            math::ray_plane(o, d, t, [0.0, 1.0, 0.0]).map(from_view)
        }
    }

    /// Orbit (dx, dy in px) or pan in the camera plane.
    pub fn orbit_pan(&mut self, dx: f32, dy: f32, pan: bool) {
        if pan {
            let e = self.cam.cur.eye();
            let f = math::norm(math::sub(self.cam.cur.target, e));
            let s = math::norm(math::cross(f, [0.0, 1.0, 0.0]));
            let u = math::cross(s, f);
            let k = self.cam.cur.dist * 1.6 / self.px[1].max(1.0);
            self.cam.pan(math::add(math::scale(s, -dx * k), math::scale(u, dy * k)));
        } else {
            self.cam.rotate(-dx * 0.008, dy * 0.008);
        }
    }

    /// Advance the camera and draw. Returns true while the camera is still moving.
    pub fn frame(&mut self, dt: f32, sc: &Scene) -> bool {
        let dpr = web_sys::window().map(|w| w.device_pixel_ratio()).unwrap_or(1.0) as f32;
        let (cw, ch) = (self.canvas.client_width() as f32, self.canvas.client_height() as f32);
        let (w, h) = ((cw * dpr) as u32, (ch * dpr) as u32);
        if self.canvas.width() != w || self.canvas.height() != h {
            self.canvas.set_width(w);
            self.canvas.set_height(h);
        }
        self.px = [cw.max(1.0), ch.max(1.0)];
        let moving = self.cam.step(dt);
        let o = self.cam.cur;
        let eye = o.eye();
        let view = math::look_at(eye, o.target, [0.0, 1.0, 0.0]);
        let proj = math::perspective(0.8, self.px[0] / self.px[1], self.size * 0.01, self.size * 60.0);
        self.vp = math::mul(&proj, &view);
        self.inv_vp = math::invert(&self.vp).unwrap_or(math::IDENTITY);
        self.r.begin(&view, &proj, eye, [0, 0, w as i32, h as i32]);
        self.draw_scene(sc);
        moving
    }

    fn draw_scene(&self, sc: &Scene) {
        let (r, s) = (&self.r, self.size);
        let up = [0.0, 1.0, 0.0];
        // Pedestal.
        let ped = math::basis([1.0, 0.0, 0.0], up, [0.0, 0.0, 1.0], [s * 0.12, s * 0.03, s * 0.12], [0.0, s * 0.015, 0.0]);
        r.draw(&self.cyl, &ped, Paint::lit([0.30, 0.32, 0.36, 1.0]));
        // Links along the DH path (d then a for standard, a then d for modified) and to the tool.
        let mut solid: Vec<(&Mesh, M4)> = vec![];
        let fr = &sc.pose.frames;
        let mut segs = vec![];
        for (i, j) in sc.arm.joints.iter().enumerate() {
            let (p, n) = (fr[i], fr[i + 1].p);
            let m = match sc.arm.conv {
                Convention::Standard => add3(p.p, p.axis(2), j.d),
                Convention::Modified => add3(p.p, p.axis(0), j.a),
            };
            segs.extend([(p.p, m), (m, n)]);
        }
        segs.push((fr.last().unwrap().p, sc.pose.tool.p));
        for (a, b) in segs {
            let (a, b) = (to_view(a), to_view(b));
            let v = math::sub(b, a);
            let l = math::dot(v, v).sqrt();
            if l > 1e-5 {
                solid.push((&self.cyl, cyl_model(a, b, s * 0.018)));
                solid.push((&self.sph, math::basis([1.0, 0.0, 0.0], up, [0.0, 0.0, 1.0], [s * 0.018; 3], b)));
            }
        }
        for m in &solid {
            r.draw(m.0, &m.1, Paint::lit([0.78, 0.80, 0.84, 1.0]));
        }
        // Joint housings, aligned to each joint axis.
        let hr = self.housing_r();
        for (i, &(c, z)) in sc.pose.axes.iter().enumerate() {
            let (c, z) = (to_view(c), to_view(z));
            let (a, b) = (math::add(c, math::scale(z, -hr * 1.1)), math::add(c, math::scale(z, hr * 1.1)));
            let m = cyl_model(a, b, hr);
            let col = if sc.sel == Some(i) { [1.0, 0.55, 0.10, 1.0] } else { [0.20, 0.42, 0.78, 1.0] };
            let glow = if self.hover == Hit::Joint(i) || self.drag == Drag::Joint(i) { 0.35 } else { 0.0 };
            r.draw(&self.cyl, &m, Paint { glow, ..Paint::lit(col) });
            solid.push((&self.cyl, m));
        }
        // Target sphere with a drop line to the ground and a line to the nearest sample.
        let t = to_view(sc.target);
        let tm = math::basis([1.0, 0.0, 0.0], up, [0.0, 0.0, 1.0], [s * 0.03; 3], t);
        let glow = if self.hover == Hit::Target || self.drag == Drag::Target { 0.4 } else { 0.0 };
        r.draw(&self.sph, &tm, Paint { glow, ..Paint::lit([0.85, 0.25, 0.65, 1.0]) });
        solid.push((&self.sph, tm));
        for (m, model) in &solid {
            r.shadow(m, model);
        }
        if let Some(c) = &self.cloud {
            if sc.show[0] {
                r.draw_first(c, &math::IDENTITY, Paint::lit([1.0; 4]), self.cloud_n[usize::from(sc.show[3])]);
            }
        }
        let pink = [1.0, 0.5, 0.85, 1.0];
        let mut drop = vec![t, [t[0], 0.0, t[2]]];
        if let Some(n) = sc.nearest {
            drop.extend([t, to_view(n)]);
        }
        if let Some(m) = r.mesh(&gl::lines(&drop)) {
            r.draw(&m, &math::IDENTITY, Paint::flat(pink));
            r.free(m);
        }
        // Frames as RGB axes: the tool always, joint frames when toggled.
        let mut show: Vec<(ar_core::Tf, f32)> = vec![(sc.pose.tool, s * 0.12)];
        if sc.show[2] {
            show.extend(fr.iter().map(|f| (*f, s * 0.06)));
        }
        for (f, l) in show {
            let m = math::basis(to_view(f.axis(0)), to_view(f.axis(1)), to_view(f.axis(2)), [l; 3], to_view(f.p));
            r.draw(&self.axes, &m, Paint::flat([1.0; 4]));
        }
        // Manipulability ellipsoid (velocity): semi-axes σ_i along the eigenvectors, scaled so the
        // largest is 20 % of the arm size.
        if sc.show[1] && sc.sigma[0] > 1e-12 {
            let k = s * 0.2 / sc.sigma[0] as f32;
            let ax = |c: usize| to_view([sc.dirs[0][c], sc.dirs[1][c], sc.dirs[2][c]]);
            let sc3 = sc.sigma.map(|x| (x as f32 * k).max(s * 0.002));
            let m = math::basis(ax(0), ax(1), ax(2), sc3, to_view(sc.pose.tool.p));
            r.draw(&self.rings, &m, Paint::flat([0.85, 0.75, 1.0, 1.0]));
            r.draw(&self.sph, &m, Paint::lit([0.62, 0.40, 0.95, 0.30]));
        }
    }
}

fn add3(p: V3, d: V3, k: f64) -> V3 {
    [p[0] + d[0] * k, p[1] + d[1] * k, p[2] + d[2] * k]
}

/// Model matrix placing the unit y-cylinder between `a` and `b` with radius `rad`.
fn cyl_model(a: V, b: V, rad: f32) -> M4 {
    let v = math::sub(b, a);
    let l = math::dot(v, v).sqrt().max(1e-6);
    let y = math::scale(v, 1.0 / l);
    let h = if y[1].abs() < 0.9 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] };
    let x = math::norm(math::cross(y, h));
    let z = math::cross(x, y);
    math::basis(x, y, z, [rad * 2.0, l, rad * 2.0], math::scale(math::add(a, b), 0.5))
}
