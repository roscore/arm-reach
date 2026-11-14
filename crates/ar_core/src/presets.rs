//! Generic "typical" arm presets. Round illustrative numbers, not taken from any datasheet.
use crate::{Arm, Convention, Joint};

pub struct Preset {
    /// Stable key, used for i18n labels in the UI.
    pub key: &'static str,
    pub arm: Arm,
}

/// Row in degrees / metres: (a, alpha°, d, offset°, min°, max°).
type Row = (f64, f64, f64, f64, f64, f64);

fn arm(conv: Convention, rows: &[Row], tool: [f64; 3]) -> Arm {
    let r = f64::to_radians;
    let joints = rows.iter().map(|&(a, al, d, off, lo, hi)| Joint { a, alpha: r(al), d, offset: r(off), min: r(lo), max: r(hi) }).collect();
    Arm { conv, joints, tool }
}

pub fn all() -> Vec<Preset> {
    vec![
        Preset {
            // Offset shoulder, elbow, and three wrist axes meeting in one point (standard DH).
            key: "industrial6",
            arm: arm(
                Convention::Standard,
                &[
                    (0.15, -90.0, 0.45, 0.0, -170.0, 170.0),
                    (0.60, 0.0, 0.0, -90.0, -100.0, 140.0),
                    (0.12, -90.0, 0.0, 0.0, -160.0, 80.0),
                    (0.0, 90.0, 0.62, 0.0, -180.0, 180.0),
                    (0.0, -90.0, 0.0, 0.0, -125.0, 125.0),
                    (0.0, 0.0, 0.10, 0.0, -350.0, 350.0),
                ],
                [0.0; 3],
            ),
        },
        Preset {
            // Spherical shoulder, elbow, spherical wrist; one redundant axis (standard DH).
            key: "cobot7",
            arm: arm(
                Convention::Standard,
                &[
                    (0.0, -90.0, 0.36, 0.0, -170.0, 170.0),
                    (0.0, 90.0, 0.0, 0.0, -120.0, 120.0),
                    (0.0, 90.0, 0.42, 0.0, -170.0, 170.0),
                    (0.0, -90.0, 0.0, 0.0, -120.0, 120.0),
                    (0.0, -90.0, 0.40, 0.0, -170.0, 170.0),
                    (0.0, 90.0, 0.0, 0.0, -120.0, 120.0),
                    (0.0, 0.0, 0.12, 0.0, -175.0, 175.0),
                ],
                [0.0; 3],
            ),
        },
        Preset {
            // Shoulder pitch-roll-yaw, elbow, forearm yaw, wrist pitch; mounted 1 m up (modified DH).
            key: "humanoid6",
            arm: arm(
                Convention::Modified,
                &[
                    (0.0, 0.0, 1.0, 0.0, -180.0, 60.0),
                    (0.0, 90.0, 0.0, 90.0, -20.0, 160.0),
                    (0.0, 90.0, 0.28, 0.0, -110.0, 110.0),
                    (0.0, -90.0, 0.0, 0.0, 0.0, 145.0),
                    (0.0, 90.0, 0.25, 0.0, -110.0, 110.0),
                    (0.0, -90.0, 0.0, 0.0, -80.0, 80.0),
                ],
                [0.0, -0.08, 0.0],
            ),
        },
    ]
}
