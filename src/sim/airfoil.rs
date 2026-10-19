//! Section coefficients of airfoils over the full range of angles of attack.

use std::f64::consts::PI;

use super::config::Airfoil;

/// Drag coefficient of a flat plate perpendicular to the flow.
const CD_90: f64 = 1.98;
/// How sharply attached flow gives way to separated flow around the stall angles, per radian.
const STALL_SHARPNESS: f64 = 50.0;
/// Range over which a table blends into flat plate behavior beyond its last entry.
const TABLE_BLEND: f64 = 10.0 * PI / 180.0;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Coefficients {
    pub cl: f64,
    pub cd: f64,
    /// Pitching moment about the quarter chord, positive nose up.
    pub cm: f64,
    /// How separated the flow is, from `0` for attached to `1` for fully separated.
    pub separation: f64,
}

#[derive(Debug, Clone)]
pub struct AirfoilModel {
    kind: Kind,
    /// Drag coefficient at zero lift, for the flat plate part of the model.
    cd_zero: f64,
}

#[derive(Debug, Clone)]
enum Kind {
    Parametric {
        lift_slope: f64,
        zero_lift_angle: f64,
        cd_min: f64,
        cd_k: f64,
        cm: f64,
        stall_positive: f64,
        stall_negative: f64,
    },
    Table {
        alpha: Vec<f64>,
        cl: Vec<f64>,
        cd: Vec<f64>,
        cm: Vec<f64>,
    },
}

impl AirfoilModel {
    pub fn new(config: &Airfoil) -> Result<Self, String> {
        match config {
            Airfoil::Parametric {
                lift_slope,
                zero_lift_angle,
                cd_min,
                cd_k,
                cm,
                stall_positive,
                stall_negative,
                ..
            } => {
                if stall_positive <= zero_lift_angle || stall_negative >= zero_lift_angle {
                    return Err(
                        "stall angles must lie on either side of the zero lift angle".into(),
                    );
                }
                Ok(Self {
                    kind: Kind::Parametric {
                        lift_slope: *lift_slope,
                        zero_lift_angle: zero_lift_angle.to_radians(),
                        cd_min: *cd_min,
                        cd_k: *cd_k,
                        cm: *cm,
                        stall_positive: stall_positive.to_radians(),
                        stall_negative: stall_negative.to_radians(),
                    },
                    cd_zero: *cd_min,
                })
            }
            Airfoil::Table {
                alpha, cl, cd, cm, ..
            } => {
                let n = alpha.len();
                if n < 2 || cl.len() != n || cd.len() != n || cm.len() != n {
                    return Err("airfoil tables need at least two rows of equal length".into());
                }
                if alpha.windows(2).any(|w| w[1] <= w[0]) {
                    return Err("airfoil table angles must be increasing".into());
                }
                Ok(Self {
                    cd_zero: cd.iter().copied().fold(f64::INFINITY, f64::min),
                    kind: Kind::Table {
                        alpha: alpha.iter().map(|a| a.to_radians()).collect(),
                        cl: cl.clone(),
                        cd: cd.clone(),
                        cm: cm.clone(),
                    },
                })
            }
        }
    }

    /// Lift slope of the attached flow per radian.
    pub fn lift_slope(&self) -> f64 {
        match &self.kind {
            Kind::Parametric { lift_slope, .. } => *lift_slope,
            Kind::Table { alpha, cl, .. } => {
                // Around the middle of the table, where the flow is attached
                let mid = alpha.len() / 2;
                let (a, b) = (mid.saturating_sub(1), (mid + 1).min(alpha.len() - 1));
                ((cl[b] - cl[a]) / (alpha[b] - alpha[a])).max(0.5)
            }
        }
    }

    /// Coefficients at an angle of attack in radians, of any magnitude.
    pub fn coefficients(&self, alpha: f64) -> Coefficients {
        let alpha = wrap_angle(alpha);
        match &self.kind {
            &Kind::Parametric {
                lift_slope,
                zero_lift_angle,
                cd_min,
                cd_k,
                cm,
                stall_positive,
                stall_negative,
            } => {
                let cl = lift_slope * (alpha - zero_lift_angle);
                let attached = Coefficients {
                    cl,
                    cd: cd_min + cd_k * cl * cl,
                    cm,
                    separation: 0.0,
                };
                let separation = separation(alpha, stall_positive, stall_negative);
                blend(attached, flat_plate(alpha, self.cd_zero), separation)
            }
            Kind::Table {
                alpha: angles,
                cl,
                cd,
                cm,
            } => {
                let (first, last) = (angles[0], angles[angles.len() - 1]);
                let clamped = alpha.clamp(first, last);
                let i = angles
                    .partition_point(|a| *a <= clamped)
                    .clamp(1, angles.len() - 1);
                let t = (clamped - angles[i - 1]) / (angles[i] - angles[i - 1]);
                let lerp = |v: &[f64]| v[i - 1] + (v[i] - v[i - 1]) * t;
                let tabled = Coefficients {
                    cl: lerp(cl),
                    cd: lerp(cd),
                    cm: lerp(cm),
                    separation: 0.0,
                };
                // Beyond the table, blend into the flat plate
                let outside = (first - alpha).max(alpha - last).max(0.0);
                let weight = smoothstep(outside / TABLE_BLEND);
                blend(tabled, flat_plate(alpha, self.cd_zero), weight)
            }
        }
    }
}

/// The incremental effect of a trailing edge flap, from thin airfoil theory.
#[derive(Debug, Clone, Copy)]
pub struct Flap {
    /// Change in effective angle of attack per radian of deflection.
    tau: f64,
    /// Change in quarter chord pitching moment per radian of deflection.
    cm_per_radian: f64,
    chord_fraction: f64,
}

impl Flap {
    pub fn new(chord_fraction: f64) -> Self {
        let cf = chord_fraction.clamp(0.01, 0.99);
        let theta = (2.0 * cf - 1.0).acos();
        Self {
            tau: 1.0 - (theta - theta.sin()) / PI,
            cm_per_radian: -0.5 * theta.sin() * (1.0 - theta.cos()),
            chord_fraction: cf,
        }
    }

    /// The change in effective angle of attack and pitching moment, and the additional drag,
    /// of a deflection in radians. Flaps lose effectiveness at large deflections, as the flow
    /// over them separates.
    pub fn effect(&self, deflection: f64) -> (f64, f64, f64) {
        let degrees = deflection.abs().to_degrees();
        let effectiveness = 1.0 - 0.5 * ((degrees - 10.0) / 40.0).clamp(0.0, 1.0);
        let alpha = self.tau * effectiveness * deflection;
        let cm = self.cm_per_radian * effectiveness * deflection;
        let cd = 0.9 * self.chord_fraction * deflection.sin().powi(2);
        (alpha, cm, cd)
    }
}

/// A flat plate with the center of pressure near the middle of its chord.
fn flat_plate(alpha: f64, cd_zero: f64) -> Coefficients {
    let normal = CD_90 * alpha.sin();
    Coefficients {
        cl: normal * alpha.cos(),
        cd: cd_zero + normal * alpha.sin(),
        cm: -0.2 * normal,
        separation: 1.0,
    }
}

/// From `0` between the stall angles to `1` beyond them, smoothly, after Beard & McLain.
fn separation(alpha: f64, positive: f64, negative: f64) -> f64 {
    let m = STALL_SHARPNESS;
    let a = (-m * (alpha - positive)).exp();
    let b = (m * (alpha - negative)).exp();
    ((1.0 + a + b) / ((1.0 + a) * (1.0 + b))).clamp(0.0, 1.0)
}

fn blend(attached: Coefficients, separated: Coefficients, weight: f64) -> Coefficients {
    let lerp = |a: f64, b: f64| a + (b - a) * weight;
    Coefficients {
        cl: lerp(attached.cl, separated.cl),
        cd: lerp(attached.cd, separated.cd),
        cm: lerp(attached.cm, separated.cm),
        separation: weight,
    }
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Wrap an angle into `-π..π`.
pub fn wrap_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::config::AirfoilShape;

    fn cambered() -> AirfoilModel {
        AirfoilModel::new(&Airfoil::Parametric {
            lift_slope: 5.7,
            zero_lift_angle: -3.0,
            cd_min: 0.01,
            cd_k: 0.01,
            cm: -0.05,
            stall_positive: 13.0,
            stall_negative: -9.0,
            shape: AirfoilShape::default(),
        })
        .unwrap()
    }

    #[test]
    fn attached_flow_is_linear() {
        let airfoil = cambered();
        let c = airfoil.coefficients(5f64.to_radians());
        assert!((c.cl - 5.7 * 8f64.to_radians()).abs() < 0.02, "{c:?}");
        assert!(c.separation < 0.01);
    }

    #[test]
    fn lift_drops_beyond_stall() {
        let airfoil = cambered();
        let peak = airfoil.coefficients(13f64.to_radians()).cl;
        let stalled = airfoil.coefficients(25f64.to_radians());
        assert!(stalled.cl < peak, "{peak} {stalled:?}");
        assert!(stalled.separation > 0.99);
        // A flat plate broadside to the flow has no lift and maximum drag
        let broadside = airfoil.coefficients(90f64.to_radians());
        assert!(broadside.cl.abs() < 1e-9 && (broadside.cd - CD_90 - 0.01).abs() < 1e-9);
    }

    #[test]
    fn continuous_over_full_circle() {
        let airfoil = cambered();
        let mut previous = airfoil.coefficients(-PI);
        for i in 1..=3600 {
            let alpha = -PI + i as f64 * 2.0 * PI / 3600.0;
            let c = airfoil.coefficients(alpha);
            assert!((c.cl - previous.cl).abs() < 0.05, "jump in cl at {alpha}");
            assert!(c.cd > 0.0);
            previous = c;
        }
    }

    #[test]
    fn flap_increases_lift() {
        let flap = Flap::new(0.25);
        let (alpha, cm, cd) = flap.effect(10f64.to_radians());
        // About 0.6 degrees of angle of attack per degree of a 25% flap
        assert!((alpha / 10f64.to_radians() - 0.6).abs() < 0.05, "{alpha}");
        assert!(cm < 0.0 && cd > 0.0);
    }

    #[test]
    fn table_interpolates() {
        let airfoil = AirfoilModel::new(&Airfoil::Table {
            alpha: vec![-10.0, 0.0, 10.0],
            cl: vec![-0.8, 0.2, 1.2],
            cd: vec![0.02, 0.01, 0.02],
            cm: vec![0.0, -0.05, -0.05],
            shape: AirfoilShape::default(),
        })
        .unwrap();
        let c = airfoil.coefficients(5f64.to_radians());
        assert!((c.cl - 0.7).abs() < 1e-9);
        let far = airfoil.coefficients(40f64.to_radians());
        assert_eq!(far.separation, 1.0);
    }
}
