//! arm-reach web UI: binds the static markup in index.html to ar_core and the 3D / 2D views.
pub mod gl;
mod i18n;
pub mod math;
mod plot2d;
mod view3d;

use ar_core::{presets, Arm, Convention, Dexterity, Joint, Pose, Samples, Stats, V3};
use i18n::{tr, Lang};
use std::{cell::RefCell, rc::Rc};
use view3d::{Drag, Hit, Scene, View};
use wasm_bindgen::{prelude::*, JsCast};
use web_sys::{Document, Element, HtmlCanvasElement, HtmlElement, HtmlInputElement, HtmlSelectElement, PointerEvent, WheelEvent};

const LANG_KEY: &str = "arm-reach.lang";
const INIT_Q: [f64; 7] = [0.2, 0.4, -0.3, 0.9, 0.3, 0.6, 0.0];
const FIELDS: [&str; 6] = ["a", "al", "d", "off", "min", "max"];

struct App {
    doc: Document,
    lang: Lang,
    dark: bool,
    arm: Arm,
    q: Vec<f64>,
    rows: usize,
    count: usize,
    frac: f64,
    samples: Samples,
    stats: Stats,
    target: V3,
    near: Option<(usize, f64)>,
    sel: Option<usize>,
    sweep: bool,
    slow: bool,
    phase: f64,
    show: [bool; 4],
    pose: Pose,
    dex: Dexterity,
    w: f64,
    view: Option<View>,
    dirty: bool,
    last_t: f64,
}

fn f3(x: f64) -> String {
    format!("{x:.3}")
}
fn deg(x: f64) -> String {
    format!("{:.1}", x.to_degrees())
}
fn kv(rows: &[(String, String)]) -> String {
    rows.iter().map(|(k, v)| format!("<b>{k}</b><span>{v}</span>")).collect()
}

impl App {
    fn el(&self, id: &str) -> HtmlElement {
        self.doc.get_element_by_id(id).and_then(|e| e.dyn_into().ok()).expect(id)
    }
    fn input(&self, id: &str) -> HtmlInputElement {
        self.el(id).unchecked_into()
    }
    fn select(&self, id: &str) -> String {
        self.el(id).unchecked_into::<HtmlSelectElement>().value()
    }
    fn set_html(&self, id: &str, h: &str) {
        self.el(id).set_inner_html(h);
    }
    fn t(&self, k: &str) -> &'static str {
        tr(self.lang, k)
    }

    fn load_preset(&mut self, k: usize) {
        self.arm = presets::all().swap_remove(k.min(2)).arm;
        self.q = self.arm.joints.iter().zip(INIT_Q).map(|(j, q)| q.clamp(j.min, j.max)).collect();
        self.sel = None;
        self.sweep = false;
        self.write_table();
        self.resample();
        let s = self.stats;
        // Start the reach target just off a sampled point, so it sits inside the workspace.
        let p = self.samples.p.get(self.samples.len() / 3).copied().unwrap_or_default();
        self.target = [p[0] + 0.03, p[1] - 0.03, p[2] + 0.03];
        if let Some(v) = &mut self.view {
            v.fit(s.max_reach, [0.0, 0.0, (s.z_min.max(0.0) + s.z_max) * 0.4]);
        }
        // Rebuild the cloud now that the point size matches this arm.
        self.restats();
        self.update_near();
    }

    fn write_table(&self) {
        let r = |x: f64| format!("{:.1}", x.to_degrees());
        let mut h = String::new();
        for (i, j) in self.arm.joints.iter().enumerate() {
            let vals = [format!("{:.3}", j.a), r(j.alpha), format!("{:.3}", j.d), r(j.offset), r(j.min), r(j.max)];
            h += &format!("<tr><td>{}</td>", i + 1);
            for (f, v) in FIELDS.iter().zip(vals) {
                let step = if f.len() == 1 { "0.01" } else { "1" };
                h += &format!("<td><input type=number step={step} data-j={i} data-f={f} value=\"{v}\"></td>");
            }
            h += "</tr>";
        }
        self.set_html("dh", &h);
        for (id, v) in ["tx", "ty", "tz"].iter().zip(self.arm.tool) {
            self.input(id).set_value(&format!("{v:.3}"));
        }
        let conv = if self.arm.conv == Convention::Standard { "std" } else { "mod" };
        self.el("conv").unchecked_into::<HtmlSelectElement>().set_value(conv);
        self.el("dof").unchecked_into::<HtmlSelectElement>().set_value(&self.arm.dof().to_string());
        self.write_sliders();
    }

    fn write_sliders(&self) {
        let mut h = String::new();
        for (i, (j, q)) in self.arm.joints.iter().zip(&self.q).enumerate() {
            let on = if self.sel == Some(i) { " class=on" } else { "" };
            h += &format!(
                "<div class=qrow><button data-act=j{i}{on}>J{}</button><input type=range data-q={i} min={} max={} step=0.1 value={}><span id=qv{i}></span></div>",
                i + 1,
                j.min.to_degrees(),
                j.max.to_degrees(),
                q.to_degrees()
            );
        }
        self.set_html("qs", &h);
        self.sync_sliders();
    }

    fn sync_sliders(&self) {
        for (i, q) in self.q.iter().enumerate() {
            if let Some(e) = self.doc.query_selector(&format!("[data-q=\"{i}\"]")).ok().flatten() {
                e.unchecked_into::<HtmlInputElement>().set_value_as_number(q.to_degrees());
            }
            self.el(&format!("qv{i}")).set_text_content(Some(&format!("{}°", deg(*q))));
        }
    }

    /// Read the DH table, tool and convention back from the form.
    fn read_table(&mut self) {
        let num = |e: &Element| e.clone().unchecked_into::<HtmlInputElement>().value_as_number();
        for e in self.doc.query_selector_all("[data-j]").map(|l| (0..l.length()).filter_map(|i| l.item(i)).collect::<Vec<_>>()).unwrap_or_default() {
            let e: Element = e.unchecked_into();
            let (Some(i), Some(f)) = (e.get_attribute("data-j").and_then(|s| s.parse::<usize>().ok()), e.get_attribute("data-f")) else { continue };
            let (v, Some(j)) = (num(&e), self.arm.joints.get_mut(i)) else { continue };
            if !v.is_finite() {
                continue;
            }
            match f.as_str() {
                "a" => j.a = v,
                "d" => j.d = v,
                "al" => j.alpha = v.to_radians(),
                "off" => j.offset = v.to_radians(),
                "min" => j.min = v.to_radians(),
                _ => j.max = v.to_radians(),
            }
            if j.min > j.max {
                std::mem::swap(&mut j.min, &mut j.max);
            }
        }
        for (k, id) in ["tx", "ty", "tz"].iter().enumerate() {
            let v = self.input(id).value_as_number();
            if v.is_finite() {
                self.arm.tool[k] = v;
            }
        }
        self.arm.conv = if self.select("conv") == "mod" { Convention::Modified } else { Convention::Standard };
        self.clamp_q();
        self.write_sliders();
    }

    fn clamp_q(&mut self) {
        self.q.resize(self.arm.dof(), 0.0);
        for (q, j) in self.q.iter_mut().zip(&self.arm.joints) {
            *q = q.clamp(j.min, j.max);
        }
    }

    fn set_dof(&mut self, n: usize) {
        let extra = Joint { a: 0.0, alpha: 0.0, d: 0.05, offset: 0.0, min: -170f64.to_radians(), max: 170f64.to_radians() };
        self.arm.joints.resize(n, extra);
        self.sel = self.sel.filter(|&s| s < n);
        self.clamp_q();
        self.sweep = false;
        self.write_table();
        self.resample();
        self.apply_lang();
    }

    fn resample(&mut self) {
        self.samples = ar_core::sample(&self.arm, self.count, ar_core::SEED, self.rows);
        self.restats();
    }

    fn restats(&mut self) {
        self.stats = ar_core::stats(&self.samples, self.frac);
        let s = self.stats;
        let mut ord: Vec<usize> = (0..self.samples.len()).collect();
        ord.sort_by_key(|&i| self.samples.w[i] >= s.w_thresh);
        let near = ord.iter().take_while(|&&i| self.samples.w[i] < s.w_thresh).count();
        if let Some(v) = &mut self.view {
            let pts: Vec<V3> = ord.iter().map(|&i| self.samples.p[i]).collect();
            let cols: Vec<[f32; 3]> = ord
                .iter()
                .map(|&i| if self.samples.w[i] < s.w_thresh { plot2d::RED } else { plot2d::colormap(self.samples.w[i] / s.w_max.max(1e-12)) })
                .collect();
            v.set_cloud(&pts, &cols, near);
        }
        self.update_pose();
        self.update_near();
        self.text_stats();
    }

    fn text_stats(&self) {
        let s = self.stats;
        let m = |x: f64| format!("{} m", f3(x));
        let h = if self.samples.is_empty() {
            kv(&[(self.t("status").into(), self.t("nodata").into())])
        } else {
            kv(&[
                (self.t("max_reach").into(), m(s.max_reach)),
                (self.t("radius").into(), format!("{} / {}", m(s.r_min), m(s.r_max))),
                (self.t("zrange").into(), format!("{} / {}", m(s.z_min), m(s.z_max))),
                (self.t("wmax").into(), format!("{:.4}", s.w_max)),
                (self.t("nearfrac").into(), format!("{:.1} %", s.near_singular * 100.0)),
            ])
        };
        self.set_html("stats", &h);
    }

    fn update_pose(&mut self) {
        self.pose = self.arm.fk(&self.q);
        let jac = self.arm.jacobian(&self.pose);
        self.dex = ar_core::dexterity(&jac);
        self.w = ar_core::manipulability(&jac, self.rows);
        self.sync_sliders();
        self.text_pose();
        self.dirty = true;
    }

    fn text_pose(&self) {
        let p = self.pose.tool.p;
        let d = self.dex;
        let margins = self.arm.margins(&self.q);
        let (mi, mv) = margins.iter().enumerate().fold((0, f64::INFINITY), |a, (i, &m)| if m < a.1 { (i, m) } else { a });
        let cond = if d.cond.is_finite() { format!("{:.1}", d.cond) } else { "∞".into() };
        let near = self.w < self.stats.w_thresh || !d.cond.is_finite();
        let badge = format!("<span class={}>{}</span>", if near { "bad" } else { "ok" }, self.t(if near { "warn" } else { "ok" }));
        let wl = if self.rows == 3 { "w (J_v)" } else { "w (J)" };
        self.set_html(
            "pose",
            &kv(&[
                (self.t("tool_xyz").into(), format!("{} / {} / {} m", f3(p[0]), f3(p[1]), f3(p[2]))),
                (wl.into(), format!("{:.4}", self.w)),
                (self.t("sigma").into(), format!("{:.4} ≥ {:.4} ≥ {:.4}", d.sigma[0], d.sigma[1], d.sigma[2])),
                (self.t("cond").into(), cond.clone()),
                (self.t("margin").into(), format!("{}° (J{})", deg(mv), mi + 1)),
                (self.t("status").into(), badge),
            ]),
        );
        self.el("hud").set_text_content(Some(&format!(
            "x {:>7} y {:>7} z {:>7}\nw     {:.4}\nσmin  {:.4}\ncond  {cond}\nlimit {}° J{}",
            f3(p[0]),
            f3(p[1]),
            f3(p[2]),
            self.w,
            d.sigma[2],
            deg(mv),
            mi + 1
        )));
        let half: Vec<f64> = self.arm.joints.iter().map(|j| (j.max - j.min).to_degrees() * 0.5).collect();
        let md: Vec<f64> = margins.iter().map(|m| m.to_degrees()).collect();
        plot2d::margins(&self.el("marg").unchecked_into(), &md, &half, self.sel, self.dark);
        let s = self.stats;
        plot2d::histogram(&self.el("hist").unchecked_into(), &self.samples.w, s.w_max, s.w_thresh, self.w, self.dark);
    }

    fn update_near(&mut self) {
        self.near = self.samples.nearest(self.target);
        for (id, v) in ["gx", "gy", "gz"].iter().zip(self.target) {
            let e = self.input(id);
            if self.doc.active_element().map(|a| a != **e).unwrap_or(true) {
                e.set_value(&format!("{v:.3}"));
            }
        }
        let h = match self.near {
            Some((i, d)) => {
                let p = self.samples.p[i];
                let q: Vec<String> = self.samples.joints(i).iter().map(|q| format!("{:.0}", q.to_degrees())).collect();
                kv(&[
                    (self.t("nearest").into(), format!("{} / {} / {} m", f3(p[0]), f3(p[1]), f3(p[2]))),
                    (self.t("dist").into(), format!("{} m", f3(d))),
                    (self.t("jvec").into(), q.join(", ")),
                ])
            }
            None => String::new(),
        };
        self.set_html("near", &h);
        self.dirty = true;
    }

    fn apply_lang(&mut self) {
        if let Ok(list) = self.doc.query_selector_all("[data-i18n]") {
            for e in (0..list.length()).filter_map(|i| list.item(i)) {
                let e: Element = e.unchecked_into();
                if let Some(k) = e.get_attribute("data-i18n") {
                    e.set_text_content(Some(self.t(&k)));
                }
            }
        }
        self.el("lang").set_text_content(Some(if self.lang == Lang::Ko { "English" } else { "한국어" }));
        if let Some(r) = self.doc.document_element() {
            r.set_attribute("lang", if self.lang == Lang::Ko { "ko" } else { "en" }).ok();
        }
        self.el("sweep").set_text_content(Some(self.t(if self.sweep { "pause" } else { "sweep" })));
        self.el("speed").set_text_content(Some(if self.slow { "0.25×" } else { "1×" }));
        self.text_stats();
        self.text_pose();
        self.update_near();
    }

    fn select_joint(&mut self, i: usize) {
        self.sel = if self.sel == Some(i) && !self.sweep { None } else { Some(i) };
        self.write_sliders();
        self.text_pose();
        self.dirty = true;
    }

    fn on_change(&mut self, e: &Element) {
        let id = e.id();
        match id.as_str() {
            "preset" => {
                self.load_preset(self.select("preset").parse().unwrap_or(0));
                self.apply_lang();
            }
            "dof" => self.set_dof(if self.select("dof") == "7" { 7 } else { 6 }),
            "count" => {
                self.count = self.select("count").parse().unwrap_or(5000);
                self.resample();
            }
            "measure" => {
                self.rows = if self.select("measure") == "6" { 6 } else { 3 };
                self.resample();
            }
            "thresh" => {
                let v = self.input("thresh").value_as_number();
                if v.is_finite() {
                    self.frac = v.clamp(0.0, 100.0) / 100.0;
                    self.restats();
                }
            }
            "gx" | "gy" | "gz" => {
                for (k, id) in ["gx", "gy", "gz"].iter().enumerate() {
                    let v = self.input(id).value_as_number();
                    if v.is_finite() {
                        self.target[k] = v;
                    }
                }
                self.update_near();
            }
            _ if id == "conv" || id.starts_with('t') || e.has_attribute("data-j") => {
                self.read_table();
                self.resample();
            }
            _ => {}
        }
    }

    fn on_input(&mut self, e: &Element) {
        if let Some(i) = e.get_attribute("data-q").and_then(|s| s.parse::<usize>().ok()) {
            let v = e.clone().unchecked_into::<HtmlInputElement>().value_as_number().to_radians();
            if let (Some(q), Some(j)) = (self.q.get_mut(i), self.arm.joints.get(i)) {
                *q = v.clamp(j.min, j.max);
                self.update_pose();
            }
        }
    }

    fn on_click(&mut self, act: &str, btn: &Element) {
        match act {
            "lang" => {
                self.lang = if self.lang == Lang::Ko { Lang::En } else { Lang::Ko };
                if let Some(s) = web_sys::window().and_then(|w| w.local_storage().ok().flatten()) {
                    s.set_item(LANG_KEY, if self.lang == Lang::Ko { "ko" } else { "en" }).ok();
                }
                self.apply_lang();
            }
            "load" => {
                if let Some((i, _)) = self.near {
                    self.q = self.samples.joints(i).to_vec();
                    self.update_pose();
                }
            }
            "sweep" => {
                self.sweep = !self.sweep;
                if self.sweep {
                    let i = *self.sel.get_or_insert(0);
                    let j = self.arm.joints[i];
                    let t = ((self.q[i] - j.min) / (j.max - j.min).max(1e-9) * 2.0 - 1.0).clamp(-1.0, 1.0);
                    self.phase = t.asin();
                    self.write_sliders();
                }
                self.apply_lang();
            }
            "speed" => {
                self.slow = !self.slow;
                self.apply_lang();
            }
            _ => {
                let k: usize = act[1..].parse().unwrap_or(0);
                match &act[..1] {
                    "t" => {
                        self.show[k.min(3)] ^= true;
                        btn.class_list().toggle("on").ok();
                        self.dirty = true;
                    }
                    "v" => {
                        if let Some(v) = &mut self.view {
                            v.preset(k);
                        }
                    }
                    "j" => self.select_joint(k),
                    _ => {}
                }
            }
        }
    }

    fn scene(&self) -> Scene<'_> {
        Scene {
            arm: &self.arm,
            pose: &self.pose,
            sigma: self.dex.sigma,
            dirs: self.dex.dirs,
            target: self.target,
            nearest: self.near.map(|(i, _)| self.samples.p[i]),
            sel: self.sel,
            show: self.show,
        }
    }

    fn tick(&mut self, now: f64) {
        let dt = ((now - self.last_t) / 1000.0).clamp(0.0, 0.1);
        self.last_t = now;
        if self.sweep {
            if let Some(i) = self.sel {
                let j = self.arm.joints[i];
                self.phase += dt * if self.slow { 0.25 } else { 1.0 } * std::f64::consts::TAU / 4.0;
                self.q[i] = (j.min + j.max) * 0.5 + (j.max - j.min) * 0.5 * self.phase.sin();
                self.update_pose();
            }
        }
        let Some(mut v) = self.view.take() else { return };
        let moving = v.cam.step(0.0) || self.dirty || !v.ptrs.is_empty();
        if moving {
            v.frame(dt as f32, &self.scene());
            self.dirty = false;
            let lbl = self.el("tlabel");
            match v.project(self.target) {
                Some((x, y)) => lbl.set_attribute("style", &format!("left:{x}px;top:{y}px")).ok(),
                None => lbl.set_attribute("style", "display:none").ok(),
            };
        }
        self.view = Some(v);
    }

    // ---- pointer interaction on the 3D canvas ----

    fn pointer_down(&mut self, e: &PointerEvent) {
        let Some(mut v) = self.view.take() else { return };
        let (x, y) = (e.offset_x() as f32, e.offset_y() as f32);
        v.ptrs.push((e.pointer_id(), x, y));
        v.cam.held = true;
        if v.ptrs.len() == 1 {
            v.drag = match v.pick(x, y, &self.scene()) {
                Hit::Target => Drag::Target,
                Hit::Joint(i) => {
                    self.sel = Some(i);
                    self.write_sliders();
                    self.text_pose();
                    Drag::Joint(i)
                }
                Hit::None if e.button() == 2 => Drag::Pan,
                Hit::None => Drag::Orbit,
            };
        } else if v.drag != Drag::Target {
            v.drag = Drag::Pan;
        }
        self.view = Some(v);
        self.dirty = true;
    }

    fn pointer_move(&mut self, e: &PointerEvent) {
        let Some(mut v) = self.view.take() else { return };
        let (x, y) = (e.offset_x() as f32, e.offset_y() as f32);
        let id = e.pointer_id();
        let Some(k) = v.ptrs.iter().position(|p| p.0 == id) else {
            let h = v.pick(x, y, &self.scene());
            if h != v.hover {
                v.hover = h;
                v.canvas.style().set_property("cursor", if h == Hit::None { "default" } else { "grab" }).ok();
                self.dirty = true;
            }
            self.view = Some(v);
            return;
        };
        let old = v.ptrs.clone();
        v.ptrs[k] = (id, x, y);
        let (dx, dy) = (x - old[k].1, y - old[k].2);
        match v.drag {
            Drag::Target => {
                let vertical = e.shift_key() || v.ptrs.len() > 1;
                if !vertical || k > 0 || e.shift_key() {
                    if let Some(t) = v.drag_target(x, y, self.target, vertical) {
                        self.target = t;
                        self.update_near();
                    }
                }
            }
            Drag::Joint(i) => {
                let j = self.arm.joints[i];
                self.q[i] = (self.q[i] + dx as f64 * 0.01).clamp(j.min, j.max);
                self.update_pose();
            }
            Drag::Orbit => v.orbit_pan(dx, dy, false),
            Drag::Pan if v.ptrs.len() >= 2 => {
                let mid = |p: &[(i32, f32, f32)]| ((p[0].1 + p[1].1) * 0.5, (p[0].2 + p[1].2) * 0.5);
                let dist = |p: &[(i32, f32, f32)]| (p[0].1 - p[1].1).hypot(p[0].2 - p[1].2).max(1.0);
                let (m0, m1) = (mid(&old), mid(&v.ptrs));
                v.cam.zoom(dist(&old) / dist(&v.ptrs));
                v.orbit_pan(m1.0 - m0.0, m1.1 - m0.1, true);
            }
            Drag::Pan => v.orbit_pan(dx, dy, true),
            Drag::None => {}
        }
        self.view = Some(v);
        self.dirty = true;
    }

    fn pointer_up(&mut self, e: &PointerEvent) {
        if let Some(v) = &mut self.view {
            v.ptrs.retain(|p| p.0 != e.pointer_id());
            if v.ptrs.is_empty() {
                v.cam.held = false;
            }
            v.drag = Drag::None;
            self.dirty = true;
        }
    }
}

fn listen<E: JsCast + 'static>(t: &web_sys::EventTarget, ev: &str, app: &Rc<RefCell<App>>, f: impl Fn(&mut App, &E) + 'static) {
    let app = app.clone();
    let cb = Closure::<dyn FnMut(web_sys::Event)>::new(move |e: web_sys::Event| f(&mut app.borrow_mut(), e.unchecked_ref::<E>()));
    t.add_event_listener_with_callback(ev, cb.as_ref().unchecked_ref()).ok();
    cb.forget();
}

#[wasm_bindgen(start)]
pub fn start() {
    let Some(win) = web_sys::window() else { return };
    let Some(doc) = win.document() else { return };
    let dark = win.match_media("(prefers-color-scheme: dark)").ok().flatten().map(|m| m.matches()).unwrap_or(false);
    let stored = win.local_storage().ok().flatten().and_then(|s| s.get_item(LANG_KEY).ok().flatten());
    let lang_s = stored.unwrap_or_else(|| win.navigator().language().unwrap_or_default());
    let lang = if lang_s.starts_with("ko") { Lang::Ko } else { Lang::En };
    let canvas: HtmlCanvasElement = doc.get_element_by_id("gl").expect("gl").unchecked_into();
    let view = View::new(canvas.clone(), dark);
    if view.is_none() {
        if let Some(e) = doc.get_element_by_id("stage") {
            e.class_list().add_1("nogl").ok();
        }
    }
    let arm = presets::all().swap_remove(0).arm;
    let pose = arm.fk(&[0.0; 6]);
    let dex = ar_core::dexterity(&arm.jacobian(&pose));
    let app = Rc::new(RefCell::new(App {
        doc: doc.clone(),
        lang,
        dark,
        arm,
        q: vec![0.0; 6],
        rows: 3,
        count: 5000,
        frac: 0.05,
        samples: Samples::default(),
        stats: Stats::default(),
        target: [0.0; 3],
        near: None,
        sel: None,
        sweep: false,
        slow: false,
        phase: 0.0,
        show: [true, true, true, false],
        pose,
        dex,
        w: 0.0,
        view,
        dirty: true,
        last_t: 0.0,
    }));
    {
        let mut a = app.borrow_mut();
        a.load_preset(0);
        a.apply_lang();
    }
    let body: web_sys::EventTarget = doc.body().expect("body").into();
    listen::<web_sys::Event>(&body, "change", &app, |a, e| {
        if let Some(t) = e.target().and_then(|t| t.dyn_into::<Element>().ok()) {
            a.on_change(&t);
        }
    });
    listen::<web_sys::Event>(&body, "input", &app, |a, e| {
        if let Some(t) = e.target().and_then(|t| t.dyn_into::<Element>().ok()) {
            a.on_input(&t);
        }
    });
    listen::<web_sys::Event>(&body, "click", &app, |a, e| {
        let btn = e.target().and_then(|t| t.dyn_into::<Element>().ok()).and_then(|t| t.closest("[data-act]").ok().flatten());
        if let Some(b) = btn {
            a.on_click(&b.get_attribute("data-act").unwrap_or_default(), &b);
        }
    });
    let cv: web_sys::EventTarget = canvas.clone().into();
    listen::<PointerEvent>(&cv, "pointerdown", &app, |a, e| {
        if let Some(t) = e.target().and_then(|t| t.dyn_into::<Element>().ok()) {
            t.set_pointer_capture(e.pointer_id()).ok();
        }
        a.pointer_down(e);
    });
    listen::<PointerEvent>(&cv, "pointermove", &app, |a, e| a.pointer_move(e));
    listen::<PointerEvent>(&cv, "pointerup", &app, |a, e| a.pointer_up(e));
    listen::<PointerEvent>(&cv, "pointercancel", &app, |a, e| a.pointer_up(e));
    listen::<WheelEvent>(&cv, "wheel", &app, |a, e| {
        e.prevent_default();
        if let Some(v) = &mut a.view {
            v.cam.zoom((e.delta_y() as f32 * 0.0015).exp());
            a.dirty = true;
        }
    });
    listen::<web_sys::Event>(&cv, "dblclick", &app, |a, _| {
        if let Some(v) = &mut a.view {
            v.preset(0);
        }
    });
    listen::<web_sys::Event>(&cv, "contextmenu", &app, |_, e| e.prevent_default());
    listen::<web_sys::Event>(&win.clone().into(), "resize", &app, |a, _| {
        a.dirty = true;
        a.text_pose();
    });
    // Animation loop.
    let f: Rc<RefCell<Option<Closure<dyn FnMut(f64)>>>> = Rc::new(RefCell::new(None));
    let g = f.clone();
    *g.borrow_mut() = Some(Closure::new(move |t: f64| {
        app.borrow_mut().tick(t);
        if let Some(w) = web_sys::window() {
            w.request_animation_frame(f.borrow().as_ref().unwrap().as_ref().unchecked_ref()).ok();
        }
    }));
    win.request_animation_frame(g.borrow().as_ref().unwrap().as_ref().unchecked_ref()).ok();
}
