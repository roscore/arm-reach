//! Arm kinematics core (pure Rust, no web dependencies).
//!
//! Textbook serial-chain kinematics for all-revolute arms: Denavit–Hartenberg forward kinematics
//! (standard and modified convention), geometric Jacobian, Yoshikawa manipulability, a small
//! symmetric Jacobi eigen solver, seeded workspace sampling, reach statistics and nearest-sample query.
//! Lengths are metres, angles radians.

pub mod presets;

pub type V3 = [f64; 3];
pub type M3 = [[f64; 3]; 3];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn len(a: V3) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Rigid transform: rotation `r` (row-major, columns are the frame axes) and translation `p`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tf {
    pub r: M3,
    pub p: V3,
}

impl Tf {
    pub const ID: Tf = Tf { r: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], p: [0.; 3] };
    pub fn mul(&self, o: &Tf) -> Tf {
        let r = std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| self.r[i][k] * o.r[k][j]).sum()));
        Tf { r, p: std::array::from_fn(|i| self.p[i] + (0..3).map(|k| self.r[i][k] * o.p[k]).sum::<f64>()) }
    }
    /// Column `c` of the rotation (the frame's x, y or z axis in the parent frame).
    pub fn axis(&self, c: usize) -> V3 {
        [self.r[0][c], self.r[1][c], self.r[2][c]]
    }
    fn rot_z(t: f64) -> Tf {
        let (s, c) = t.sin_cos();
        Tf { r: [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]], ..Tf::ID }
    }
    fn rot_x(t: f64) -> Tf {
        let (s, c) = t.sin_cos();
        Tf { r: [[1., 0., 0.], [0., c, -s], [0., s, c]], ..Tf::ID }
    }
    fn trans(p: V3) -> Tf {
        Tf { p, ..Tf::ID }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Convention {
    /// T_i = Rz(θ_i) Tz(d_i) Tx(a_i) Rx(α_i); joint i turns about z of frame i−1.
    Standard,
    /// Craig: T_i = Rx(α_{i−1}) Tx(a_{i−1}) Rz(θ_i) Tz(d_i); joint i turns about z of frame i.
    Modified,
}

/// One DH row (revolute joint). θ = q + `offset`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Joint {
    pub a: f64,
    pub alpha: f64,
    pub d: f64,
    pub offset: f64,
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Arm {
    pub conv: Convention,
    pub joints: Vec<Joint>,
    /// Fixed tool point in the last joint frame.
    pub tool: V3,
}

/// Forward-kinematics result in the base frame.
#[derive(Clone, Debug)]
pub struct Pose {
    /// Base frame, then the frame after each joint (N + 1 entries).
    pub frames: Vec<Tf>,
    /// Tool frame (last frame translated by the tool offset).
    pub tool: Tf,
    /// Per joint: (point on the axis, unit axis direction).
    pub axes: Vec<(V3, V3)>,
}

impl Arm {
    pub fn dof(&self) -> usize {
        self.joints.len()
    }
    fn link(&self, j: &Joint, q: f64) -> Tf {
        let th = Tf::rot_z(q + j.offset);
        match self.conv {
            Convention::Standard => th.mul(&Tf::trans([j.a, 0., j.d])).mul(&Tf::rot_x(j.alpha)),
            Convention::Modified => Tf::rot_x(j.alpha).mul(&Tf::trans([j.a, 0., 0.])).mul(&th).mul(&Tf::trans([0., 0., j.d])),
        }
    }
    pub fn fk(&self, q: &[f64]) -> Pose {
        let mut frames = vec![Tf::ID];
        let mut axes = Vec::with_capacity(q.len());
        for (j, &qi) in self.joints.iter().zip(q) {
            let prev = *frames.last().unwrap();
            let next = prev.mul(&self.link(j, qi));
            let f = if self.conv == Convention::Standard { prev } else { next };
            axes.push((f.p, f.axis(2)));
            frames.push(next);
        }
        let tool = frames.last().unwrap().mul(&Tf::trans(self.tool));
        Pose { frames, tool, axes }
    }
    /// Geometric Jacobian, 6 rows (v then ω) × N columns: column i = [z_i × (p − o_i); z_i].
    pub fn jacobian(&self, pose: &Pose) -> Vec<[f64; 6]> {
        pose.axes
            .iter()
            .map(|&(o, z)| {
                let v = cross(z, sub(pose.tool.p, o));
                [v[0], v[1], v[2], z[0], z[1], z[2]]
            })
            .collect()
    }
    /// Per-joint distance to the nearer limit (rad, negative when outside).
    pub fn margins(&self, q: &[f64]) -> Vec<f64> {
        self.joints.iter().zip(q).map(|(j, &qi)| (qi - j.min).min(j.max - qi)).collect()
    }
    /// Upper bound on reach: sum of every row's link length plus the tool offset.
    pub fn link_sum(&self) -> f64 {
        self.joints.iter().map(|j| j.a.hypot(j.d)).sum::<f64>() + len(self.tool)
    }
}

/// Determinant of a k×k matrix (row-major, Gaussian elimination with partial pivoting).
pub fn det(mut m: Vec<f64>, k: usize) -> f64 {
    let mut d = 1.0;
    for c in 0..k {
        let p = (c..k).max_by(|&a, &b| m[a * k + c].abs().total_cmp(&m[b * k + c].abs())).unwrap();
        if m[p * k + c] == 0.0 {
            return 0.0;
        }
        if p != c {
            (0..k).for_each(|j| m.swap(p * k + j, c * k + j));
            d = -d;
        }
        d *= m[c * k + c];
        for r in c + 1..k {
            let f = m[r * k + c] / m[c * k + c];
            (c..k).for_each(|j| m[r * k + j] -= f * m[c * k + j]);
        }
    }
    d
}

/// Yoshikawa manipulability of the first `rows` Jacobian rows (3 = translational, 6 = full):
/// sqrt(det(J·Jᵀ)), or sqrt(det(Jᵀ·J)) when there are fewer joints than rows. Either way it is the
/// product of J's singular values.
pub fn manipulability(jac: &[[f64; 6]], rows: usize) -> f64 {
    let n = jac.len();
    let (k, g): (usize, Vec<f64>) = if n >= rows {
        (rows, (0..rows * rows).map(|e| jac.iter().map(|c| c[e / rows] * c[e % rows]).sum()).collect())
    } else {
        (n, (0..n * n).map(|e| (0..rows).map(|r| jac[e / n][r] * jac[e % n][r]).sum()).collect())
    };
    det(g, k).max(0.0).sqrt()
}

/// J_v·J_vᵀ (3×3, symmetric).
pub fn jv_jvt(jac: &[[f64; 6]]) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| jac.iter().map(|c| c[i] * c[j]).sum()))
}

/// Cyclic Jacobi eigen decomposition of a symmetric 3×3: eigenvalues (descending) and matching unit
/// eigenvectors (as columns of the returned matrix).
pub fn eigen_sym3(mut a: M3) -> (V3, M3) {
    let mut v = Tf::ID.r;
    for _ in 0..50 {
        let off = a[0][1].powi(2) + a[0][2].powi(2) + a[1][2].powi(2);
        if off < 1e-30 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = th.signum() / (th.abs() + (th * th + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            // a ← Gᵀ a G, v ← v G with the rotation G in the (p, q) plane.
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
                let (vkp, vkq) = (v[k][p], v[k][q]);
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    let mut ord = [0, 1, 2];
    ord.sort_by(|&i, &j| a[j][j].total_cmp(&a[i][i]));
    (ord.map(|i| a[i][i]), std::array::from_fn(|r| ord.map(|i| v[r][i])))
}

/// Translational analysis of a pose: singular values σ1 ≥ σ2 ≥ σ3 of J_v, their directions and
/// the condition number σ1/σ3 (infinite at a singularity).
#[derive(Clone, Copy, Debug)]
pub struct Dexterity {
    pub sigma: V3,
    pub dirs: M3,
    pub cond: f64,
}

pub fn dexterity(jac: &[[f64; 6]]) -> Dexterity {
    let (l, dirs) = eigen_sym3(jv_jvt(jac));
    let sigma = l.map(|x| x.max(0.0).sqrt());
    let cond = if sigma[2] > 1e-12 * sigma[0].max(1e-300) { sigma[0] / sigma[2] } else { f64::INFINITY };
    Dexterity { sigma, dirs, cond }
}

/// SplitMix64 PRNG (small, seedable, reproducible).
pub struct Rng(pub u64);

impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in [0, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

pub const SEED: u64 = 0x5EED_A12E_ACC0_0001;

/// Sampled workspace: joint vectors (flat, stride `dof`), tool positions and manipulability.
#[derive(Clone, Debug, Default)]
pub struct Samples {
    pub dof: usize,
    pub q: Vec<f64>,
    pub p: Vec<V3>,
    pub w: Vec<f64>,
}

impl Samples {
    pub fn len(&self) -> usize {
        self.p.len()
    }
    pub fn is_empty(&self) -> bool {
        self.p.is_empty()
    }
    pub fn joints(&self, i: usize) -> &[f64] {
        &self.q[i * self.dof..(i + 1) * self.dof]
    }
    /// Index of the sample whose tool position is closest to `t`.
    pub fn nearest(&self, t: V3) -> Option<(usize, f64)> {
        self.p.iter().enumerate().map(|(i, &p)| (i, len(sub(p, t)))).min_by(|a, b| a.1.total_cmp(&b.1))
    }
}

/// Uniform random joint vectors inside the limits; `rows` selects the manipulability measure.
pub fn sample(arm: &Arm, count: usize, seed: u64, rows: usize) -> Samples {
    let mut rng = Rng(seed);
    let mut s = Samples { dof: arm.dof(), ..Samples::default() };
    let mut q = vec![0.0; arm.dof()];
    for _ in 0..count {
        for (qi, j) in q.iter_mut().zip(&arm.joints) {
            *qi = j.min + (j.max - j.min) * rng.unit();
        }
        let pose = arm.fk(&q);
        s.w.push(manipulability(&arm.jacobian(&pose), rows));
        s.p.push(pose.tool.p);
        s.q.extend_from_slice(&q);
    }
    s
}

/// Reach envelope and singularity statistics (base at the origin, z up).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stats {
    pub max_reach: f64,
    pub r_min: f64,
    pub r_max: f64,
    pub z_min: f64,
    pub z_max: f64,
    pub w_max: f64,
    /// Absolute near-singular threshold (fraction × w_max).
    pub w_thresh: f64,
    /// Share of samples with w below the threshold (0..1).
    pub near_singular: f64,
}

/// `frac` is the near-singular threshold as a fraction of the maximum sampled w (e.g. 0.05).
/// Radius is horizontal distance from the base axis; max reach is 3D distance from the base origin.
pub fn stats(s: &Samples, frac: f64) -> Stats {
    if s.is_empty() {
        return Stats::default();
    }
    let fold = |f: fn(&V3) -> f64| s.p.iter().map(f).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)));
    let (r_min, r_max) = fold(|p| p[0].hypot(p[1]));
    let (z_min, z_max) = fold(|p| p[2]);
    let max_reach = s.p.iter().map(|&p| len(p)).fold(0.0, f64::max);
    let w_max = s.w.iter().copied().fold(0.0, f64::max);
    let w_thresh = frac * w_max;
    let near = s.w.iter().filter(|&&w| w < w_thresh).count();
    Stats { max_reach, r_min, r_max, z_min, z_max, w_max, w_thresh, near_singular: near as f64 / s.len() as f64 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn planar(conv: Convention, rows: [(f64, f64); 2], tool: V3) -> Arm {
        let j = |(a, d): (f64, f64)| Joint { a, alpha: 0.0, d, offset: 0.0, min: -3.0, max: 3.0 };
        Arm { conv, joints: rows.map(j).to_vec(), tool }
    }
    fn close(a: V3, b: V3, tol: f64) {
        assert!(len(sub(a, b)) < tol, "{a:?} vs {b:?}");
    }
    fn random_arm(conv: Convention, rng: &mut Rng) -> Arm {
        let mut r = |lo: f64, hi: f64| lo + (hi - lo) * rng.unit();
        let joints = (0..6).map(|_| Joint { a: r(-0.5, 0.5), alpha: r(-3.0, 3.0), d: r(-0.5, 0.5), offset: r(-1.0, 1.0), min: -3.0, max: 3.0 }).collect();
        Arm { conv, joints, tool: [r(-0.2, 0.2), r(-0.2, 0.2), r(0.0, 0.2)] }
    }

    #[test]
    fn planar_two_link_fk() {
        let arm = planar(Convention::Standard, [(1.0, 0.0), (1.0, 0.0)], [0.0; 3]);
        close(arm.fk(&[0.0, 0.0]).tool.p, [2.0, 0.0, 0.0], 1e-12);
        close(arm.fk(&[0.0, FRAC_PI_2]).tool.p, [1.0, 1.0, 0.0], 1e-12);
    }

    #[test]
    fn planar_two_link_manipulability() {
        let arm = planar(Convention::Standard, [(1.0, 0.0), (1.0, 0.0)], [0.0; 3]);
        let w = |q2: f64| manipulability(&arm.jacobian(&arm.fk(&[0.3, q2])), 3);
        assert!((w(FRAC_PI_2) - 1.0).abs() < 1e-12);
        assert!(w(0.0).abs() < 1e-9);
        assert!((w(0.7) - 0.7f64.sin()).abs() < 1e-12);
    }

    #[test]
    fn jacobian_matches_finite_differences() {
        let mut rng = Rng(7);
        for conv in [Convention::Standard, Convention::Modified] {
            let arm = random_arm(conv, &mut rng);
            let q: Vec<f64> = (0..6).map(|_| rng.unit() * 4.0 - 2.0).collect();
            let pose = arm.fk(&q);
            let jac = arm.jacobian(&pose);
            let h = 1e-6;
            for i in 0..6 {
                let (mut qp, mut qm) = (q.clone(), q.clone());
                qp[i] += h;
                qm[i] -= h;
                let (tp, tm) = (arm.fk(&qp).tool, arm.fk(&qm).tool);
                let v = std::array::from_fn(|k| (tp.p[k] - tm.p[k]) / (2.0 * h));
                close([jac[i][0], jac[i][1], jac[i][2]], v, 1e-6);
                // ω from the skew part of dR·Rᵀ.
                let d: M3 = std::array::from_fn(|r| std::array::from_fn(|c| (0..3).map(|k| (tp.r[r][k] - tm.r[r][k]) / (2.0 * h) * pose.tool.r[c][k]).sum()));
                close([jac[i][3], jac[i][4], jac[i][5]], [d[2][1], d[0][2], d[1][0]], 1e-6);
            }
        }
    }

    #[test]
    fn standard_and_modified_agree_on_converted_arm() {
        let std_arm = planar(Convention::Standard, [(1.0, 0.0), (0.8, 0.0)], [0.0; 3]);
        let mod_arm = planar(Convention::Modified, [(0.0, 0.0), (1.0, 0.0)], [0.8, 0.0, 0.0]);
        for q in [[0.0, 0.0], [0.4, -1.1], [2.0, 0.9]] {
            close(std_arm.fk(&q).tool.p, mod_arm.fk(&q).tool.p, 1e-12);
            let (a, b) = (std_arm.jacobian(&std_arm.fk(&q)), mod_arm.jacobian(&mod_arm.fk(&q)));
            assert!((manipulability(&a, 3) - manipulability(&b, 3)).abs() < 1e-12);
        }
    }

    #[test]
    fn jacobi_recovers_rotated_diagonal() {
        let (s, c) = 0.6f64.sin_cos();
        let r: M3 = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]];
        let r = Tf { r, ..Tf::ID }.mul(&Tf { r: Tf::rot_x(0.9).r, ..Tf::ID }).r;
        let l = [3.0, 1.0, 0.25];
        let a: M3 = std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| r[i][k] * l[k] * r[j][k]).sum()));
        let (e, v) = eigen_sym3(a);
        (0..3).for_each(|i| assert!((e[i] - l[i]).abs() < 1e-9, "{e:?}"));
        for k in 0..3 {
            let col = [v[0][k], v[1][k], v[2][k]];
            let av: V3 = std::array::from_fn(|i| (0..3).map(|j| a[i][j] * col[j]).sum());
            close(av, col.map(|x| x * e[k]), 1e-9);
        }
    }

    #[test]
    fn sampler_respects_limits_and_is_reproducible() {
        let arm = presets::all()[1].arm.clone();
        let a = sample(&arm, 500, SEED, 3);
        let b = sample(&arm, 500, SEED, 3);
        assert_eq!(a.q, b.q);
        assert_eq!(a.w, b.w);
        for i in 0..a.len() {
            for (q, j) in a.joints(i).iter().zip(&arm.joints) {
                assert!(*q >= j.min && *q <= j.max);
            }
        }
        assert_ne!(sample(&arm, 500, SEED + 1, 3).q, a.q);
    }

    #[test]
    fn max_reach_bounded_by_link_lengths() {
        for p in presets::all() {
            let st = stats(&sample(&p.arm, 2000, SEED, 3), 0.05);
            assert!(st.max_reach <= p.arm.link_sum() + 1e-12, "{}", p.key);
            assert!(st.max_reach > 0.3 * p.arm.link_sum(), "{}", p.key);
            assert!(st.r_min <= st.r_max && st.z_min <= st.z_max);
            assert!(st.near_singular > 0.0 && st.near_singular < 0.5, "{} {}", p.key, st.near_singular);
        }
    }

    #[test]
    fn nearest_returns_exact_sample() {
        let s = sample(&presets::all()[0].arm, 1000, SEED, 6);
        for i in [0, 17, 999] {
            let (j, d) = s.nearest(s.p[i]).unwrap();
            assert_eq!(j, i);
            assert_eq!(d, 0.0);
        }
    }

    #[test]
    fn full_manipulability_and_determinant() {
        assert!((det(vec![2., 1., 0., 1., 3., 1., 0., 1., 4.], 3) - 18.0).abs() < 1e-12);
        let arm = &presets::all()[0].arm;
        let pose = arm.fk(&[0.1, -0.4, 0.3, 0.5, 0.8, 0.2]);
        let jac = arm.jacobian(&pose);
        let d = dexterity(&jac);
        // Translational w equals the product of the singular values of J_v.
        assert!((manipulability(&jac, 3) - d.sigma.iter().product::<f64>()).abs() < 1e-9);
        assert!(manipulability(&jac, 6) > 0.0);
        assert!(d.sigma[0] >= d.sigma[1] && d.sigma[1] >= d.sigma[2] && d.cond >= 1.0);
        // Wrist singularity (q5 = 0 lines up the joint 4 and 6 axes) zeroes the full measure.
        let jac0 = arm.jacobian(&arm.fk(&[0.1, -0.4, 0.3, 0.5, 0.0, 0.2]));
        assert!(manipulability(&jac0, 6) < 1e-9);
    }
}
