//! Small canvas-2D plots: manipulability histogram and joint-limit margin bars.
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d as C2, HtmlCanvasElement};

pub const RED: [f32; 3] = [0.92, 0.18, 0.16];

/// Blue (low) → cyan → green → yellow (high) for t in 0..1.
pub fn colormap(t: f64) -> [f32; 3] {
    let stops = [[0.15, 0.30, 0.95], [0.10, 0.75, 0.90], [0.35, 0.85, 0.35], [1.00, 0.88, 0.10]];
    let x = t.clamp(0.0, 1.0) as f32 * 3.0;
    let i = (x as usize).min(2);
    let f = x - i as f32;
    std::array::from_fn(|k| stops[i][k] + (stops[i + 1][k] - stops[i][k]) * f)
}

pub fn css(c: [f32; 3]) -> String {
    format!("rgb({},{},{})", (c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8)
}

/// Fit the backing store to the CSS size × devicePixelRatio; returns the context in CSS pixels.
fn prep(cv: &HtmlCanvasElement, dark: bool) -> Option<(C2, f64, f64)> {
    let dpr = web_sys::window()?.device_pixel_ratio();
    let (w, h) = (cv.client_width() as f64, cv.client_height() as f64);
    if w < 10.0 {
        return None;
    }
    cv.set_width((w * dpr) as u32);
    cv.set_height((h * dpr) as u32);
    let c: C2 = cv.get_context("2d").ok()??.dyn_into().ok()?;
    c.scale(dpr, dpr).ok()?;
    c.set_font("11px system-ui, sans-serif");
    c.set_fill_style_str(if dark { "#9aa3b2" } else { "#5d6573" });
    c.set_stroke_style_str(if dark { "#2e3440" } else { "#d5d9e0" });
    Some((c, w, h))
}

/// Histogram of `w` over [0, w_max]; bins below `thresh` red; vertical marker at `cur`.
pub fn histogram(cv: &HtmlCanvasElement, w: &[f64], w_max: f64, thresh: f64, cur: f64, dark: bool) {
    let Some((c, cw, ch)) = prep(cv, dark) else { return };
    let (l, b) = (8.0, ch - 18.0);
    let pw = cw - 2.0 * l;
    if w.is_empty() || w_max <= 0.0 {
        return;
    }
    const N: usize = 40;
    let mut bins = [0u32; N];
    w.iter().for_each(|&x| bins[((x / w_max * N as f64) as usize).min(N - 1)] += 1);
    let top = *bins.iter().max().unwrap_or(&1) as f64;
    let bw = pw / N as f64;
    for (i, &n) in bins.iter().enumerate() {
        let mid = (i as f64 + 0.5) / N as f64;
        c.set_fill_style_str(&css(if mid * w_max < thresh { RED } else { colormap(mid) }));
        let hh = n as f64 / top * (b - 6.0);
        c.fill_rect(l + i as f64 * bw + 0.5, b - hh, bw - 1.0, hh);
    }
    c.set_fill_style_str(if dark { "#9aa3b2" } else { "#5d6573" });
    c.fill_text("0", l, ch - 4.0).ok();
    let s = format!("w = {w_max:.3}");
    c.fill_text(&s, l + pw - c.measure_text(&s).map(|m| m.width()).unwrap_or(40.0), ch - 4.0).ok();
    let x = l + (cur / w_max).clamp(0.0, 1.0) * pw;
    c.set_fill_style_str(if dark { "#ffffff" } else { "#111111" });
    c.fill_rect(x - 1.0, 2.0, 2.0, b - 2.0);
    c.fill_text(&format!("{cur:.3}"), (x + 4.0).min(cw - 40.0), 12.0).ok();
}

/// One bar per joint: margin (deg) relative to its half range; red under 10°, `sel` outlined.
pub fn margins(cv: &HtmlCanvasElement, m: &[f64], half: &[f64], sel: Option<usize>, dark: bool) {
    let Some((c, cw, ch)) = prep(cv, dark) else { return };
    let n = m.len().max(1) as f64;
    let (l, b) = (4.0, ch - 18.0);
    let bw = (cw - 2.0 * l) / n;
    for (i, (&mi, &hi)) in m.iter().zip(half).enumerate() {
        let t = (mi / hi.max(1e-9)).clamp(0.0, 1.0);
        let x = l + i as f64 * bw;
        let hh = t * (b - 16.0);
        c.set_fill_style_str(&css(if mi < 10.0 { RED } else { colormap(t) }));
        c.fill_rect(x + 3.0, b - hh, bw - 6.0, hh.max(1.0));
        if sel == Some(i) {
            c.set_line_width(2.0);
            c.set_stroke_style_str(if dark { "#60a5fa" } else { "#2563eb" });
            c.stroke_rect(x + 2.0, 1.0, bw - 4.0, b - 1.0);
        }
        c.set_fill_style_str(if dark { "#9aa3b2" } else { "#5d6573" });
        c.fill_text(&format!("J{}", i + 1), x + bw / 2.0 - 7.0, ch - 4.0).ok();
        c.fill_text(&format!("{mi:.0}"), x + bw / 2.0 - 8.0, b - hh - 3.0).ok();
    }
}
