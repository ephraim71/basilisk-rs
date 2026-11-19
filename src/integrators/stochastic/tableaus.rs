//! Coefficient tableaus of the stochastic Runge-Kutta families.
//!
//! Upstream expresses the Roessler strong methods as generic tableaus
//! (`SRACoefficients`, `SRICoefficients`) and the Tang & Xiao weak methods as
//! an `s`-stage tableau (`W2ItoCoefficients`); those are [`SraTableau`],
//! [`SriTableau`] and [`W2ItoTableau`] here, with `validate` porting the
//! constructor checks of upstream `stochasticTableauValidation.h`. The fixed
//! three- and four-stage families upstream writes with named coefficients
//! (`DRI1Coefficients`, `RSCoefficients`, `SIESMECoefficients`) keep those
//! names.
//!
//! Matrix entries follow upstream's `[stage i][sub-term j]` convention; a
//! row `i` may only use the stages `j < i` before it.

/// Correctly rounded `sqrt(6)`, so the tableaus that use it can be `const`.
const SQRT_6: f64 = 2.449_489_742_783_178_098_197_284_074_705_891_4;

/// Rejects a non-finite coefficient: upstream `stochastic_tableau::validateFinite`.
fn validate_finite(name: &str, coefficients: &[f64]) -> Result<(), String> {
    if coefficients.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(format!(
            "stochastic Runge-Kutta coefficients must all be finite ({name})"
        ))
    }
}

/// Requires finite coefficients and a zero diagonal and upper triangle, so the
/// stages are explicit: upstream `stochastic_tableau::validateExplicitMatrix`.
fn validate_explicit<const S: usize>(name: &str, matrix: &[[f64; S]; S]) -> Result<(), String> {
    for (row_index, row) in matrix.iter().enumerate() {
        validate_finite(name, row)?;
        if row[row_index..].iter().any(|&value| value != 0.0) {
            return Err(format!(
                "explicit stochastic Runge-Kutta coefficient matrices must be strictly lower \
                 triangular ({name})"
            ));
        }
    }
    Ok(())
}

/// Roessler SRA tableau for additive noise: upstream `SRACoefficients`.
///
/// ```text
/// c0 | A0 | B0
/// c1 |    |
/// ---+----+----------
///    | α  | β1  β2
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SraTableau<const S: usize> {
    /// Upstream `A0`: drift coefficients of the stages.
    pub a0: [[f64; S]; S],
    /// Upstream `B0`: diffusion coefficients of the stages (multiply `χ2`).
    pub b0: [[f64; S]; S],
    /// Upstream `alpha`: drift weights.
    pub alpha: [f64; S],
    /// Upstream `beta1`: diffusion weights on `ΔW`.
    pub beta1: [f64; S],
    /// Upstream `beta2`: diffusion weights on `χ2 = I(1,0)/h`.
    pub beta2: [f64; S],
    /// Upstream `c0`: drift stage nodes.
    pub c0: [f64; S],
    /// Upstream `c1`: nodes at which the state-independent diffusion is evaluated.
    pub c1: [f64; S],
}

impl<const S: usize> SraTableau<S> {
    /// All-zero tableau, for building a custom one.
    pub const ZERO: Self = Self {
        a0: [[0.0; S]; S],
        b0: [[0.0; S]; S],
        alpha: [0.0; S],
        beta1: [0.0; S],
        beta2: [0.0; S],
        c0: [0.0; S],
        c1: [0.0; S],
    };

    /// Upstream constructor checks of `svIntegratorStrongStochasticRungeKuttaSRA`.
    pub fn validate(&self) -> Result<(), String> {
        validate_explicit("A0", &self.a0)?;
        validate_explicit("B0", &self.b0)?;
        validate_finite("alpha", &self.alpha)?;
        validate_finite("beta1", &self.beta1)?;
        validate_finite("beta2", &self.beta2)?;
        validate_finite("c0", &self.c0)?;
        validate_finite("c1", &self.c1)
    }
}

/// Upstream `svStochasticIntegratorSRA1` coefficients (Roessler 2010).
pub const SRA1: SraTableau<2> = SraTableau {
    a0: [[0.0, 0.0], [3.0 / 4.0, 0.0]],
    b0: [[0.0, 0.0], [3.0 / 2.0, 0.0]],
    alpha: [1.0 / 3.0, 2.0 / 3.0],
    beta1: [1.0, 0.0],
    beta2: [-1.0, 1.0],
    c0: [0.0, 3.0 / 4.0],
    c1: [1.0, 0.0],
};

/// Upstream `svStochasticIntegratorSOSRA` coefficients: the
/// stability-optimized three-stage SRA.
pub const SOSRA: SraTableau<3> = SraTableau {
    a0: [
        [0.0, 0.0, 0.0],
        [0.692_396_237_615_950_7, 0.0, 0.0],
        [-3.160_914_225_282_839_5, 4.160_914_225_282_839_5, 0.0],
    ],
    b0: [
        [0.0, 0.0, 0.0],
        [1.337_163_270_439_976_3, 0.0, 0.0],
        [1.442_371_048_468_624, 1.863_274_150_113_922_5, 0.0],
    ],
    alpha: [
        0.288_987_496_689_288_5,
        0.685_988_044_083_993_7,
        0.025_024_459_226_717_772,
    ],
    beta1: [
        -16.792_534_242_221_663,
        17.514_995_785_380_226,
        0.277_538_456_841_438_35,
    ],
    beta2: [
        0.423_753_576_906_927_4,
        0.601_038_147_442_853_9,
        -1.024_791_724_349_781_3,
    ],
    c0: [0.0, 0.692_396_237_615_950_7, 1.0],
    c1: [0.0, 0.041_248_171_110_700_504, 1.0],
};

/// Roessler SRI tableau for diagonal or scalar noise: upstream
/// `SRICoefficients`.
///
/// ```text
/// c0 | A0 | B0
/// c1 | A1 | B1
/// ---+----+-------------------
///    | α  | β1  β2  β3  β4
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SriTableau<const S: usize> {
    /// Upstream `A0`: drift coefficients of the drift stages `H0`.
    pub a0: [[f64; S]; S],
    /// Upstream `A1`: drift coefficients of the diffusion stages `H1`.
    pub a1: [[f64; S]; S],
    /// Upstream `B0`: diffusion coefficients of the drift stages (multiply `χ2`).
    pub b0: [[f64; S]; S],
    /// Upstream `B1`: diffusion coefficients of the diffusion stages (multiply `√h`).
    pub b1: [[f64; S]; S],
    /// Upstream `alpha`: drift weights.
    pub alpha: [f64; S],
    /// Upstream `beta1`: diffusion weights on `ΔW`.
    pub beta1: [f64; S],
    /// Upstream `beta2`: diffusion weights on `I(1,1)/√h`.
    pub beta2: [f64; S],
    /// Upstream `beta3`: diffusion weights on `I(1,0)/h`.
    pub beta3: [f64; S],
    /// Upstream `beta4`: diffusion weights on `I(1,1,1)/h`.
    pub beta4: [f64; S],
    /// Upstream `c0`: drift stage nodes (row sums of `A0`).
    pub c0: [f64; S],
    /// Upstream `c1`: diffusion stage nodes (row sums of `A1`).
    pub c1: [f64; S],
}

impl<const S: usize> SriTableau<S> {
    /// All-zero tableau, for building a custom one.
    pub const ZERO: Self = Self {
        a0: [[0.0; S]; S],
        a1: [[0.0; S]; S],
        b0: [[0.0; S]; S],
        b1: [[0.0; S]; S],
        alpha: [0.0; S],
        beta1: [0.0; S],
        beta2: [0.0; S],
        beta3: [0.0; S],
        beta4: [0.0; S],
        c0: [0.0; S],
        c1: [0.0; S],
    };

    /// Upstream constructor checks of `svIntegratorStrongStochasticRungeKuttaSRI`.
    pub fn validate(&self) -> Result<(), String> {
        validate_explicit("A0", &self.a0)?;
        validate_explicit("A1", &self.a1)?;
        validate_explicit("B0", &self.b0)?;
        validate_explicit("B1", &self.b1)?;
        validate_finite("alpha", &self.alpha)?;
        validate_finite("beta1", &self.beta1)?;
        validate_finite("beta2", &self.beta2)?;
        validate_finite("beta3", &self.beta3)?;
        validate_finite("beta4", &self.beta4)?;
        validate_finite("c0", &self.c0)?;
        validate_finite("c1", &self.c1)
    }
}

/// Upstream `svStochasticIntegratorSRIW1` coefficients (Roessler 2010).
pub const SRIW1: SriTableau<4> = SriTableau {
    a0: [
        [0.0, 0.0, 0.0, 0.0],
        [3.0 / 4.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ],
    a1: [
        [0.0, 0.0, 0.0, 0.0],
        [1.0 / 4.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0 / 4.0, 0.0],
    ],
    b0: [
        [0.0, 0.0, 0.0, 0.0],
        [3.0 / 2.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ],
    b1: [
        [0.0, 0.0, 0.0, 0.0],
        [1.0 / 2.0, 0.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [-5.0, 3.0, 1.0 / 2.0, 0.0],
    ],
    alpha: [1.0 / 3.0, 2.0 / 3.0, 0.0, 0.0],
    beta1: [-1.0, 4.0 / 3.0, 2.0 / 3.0, 0.0],
    beta2: [-1.0, 4.0 / 3.0, -1.0 / 3.0, 0.0],
    beta3: [2.0, -4.0 / 3.0, -2.0 / 3.0, 0.0],
    beta4: [-2.0, 5.0 / 3.0, -2.0 / 3.0, 1.0],
    c0: [0.0, 3.0 / 4.0, 0.0, 0.0],
    c1: [0.0, 1.0 / 4.0, 1.0, 1.0 / 4.0],
};

/// Upstream `svStochasticIntegratorSOSRI` coefficients: the
/// stability-optimized four-stage SRI.
pub const SOSRI: SriTableau<4> = SriTableau {
    a0: [
        [0.0, 0.0, 0.0, 0.0],
        [-0.041_992_244_213_164_68, 0.0, 0.0, 0.0],
        [2.842_612_915_017_106, -2.052_772_368_400_072_7, 0.0, 0.0],
        [
            4.338_237_071_435_815,
            -2.889_593_613_743_979_3,
            2.301_757_559_464_446_6,
            0.0,
        ],
    ],
    a1: [
        [0.0, 0.0, 0.0, 0.0],
        [0.262_042_820_913_304_66, 0.0, 0.0, 0.0],
        [0.209_036_463_835_053_75, -0.150_237_711_515_036_1, 0.0, 0.0],
        [
            0.058_365_953_127_469_99,
            0.614_944_039_633_237_3,
            0.085_351_176_340_467_72,
            0.0,
        ],
    ],
    b0: [
        [0.0, 0.0, 0.0, 0.0],
        [-0.216_410_935_496_125_28, 0.0, 0.0, 0.0],
        [1.533_635_286_367_957_2, 0.260_662_234_926_470_56, 0.0, 0.0],
        [
            -1.053_603_755_817_915_9,
            1.701_528_472_108_947_2,
            -0.207_256_857_841_800_17,
            0.0,
        ],
    ],
    b1: [
        [0.0, 0.0, 0.0, 0.0],
        [-0.511_901_182_762_165_7, 0.0, 0.0, 0.0],
        [2.677_673_398_667_13, -4.939_503_132_225_099_5, 0.0, 0.0],
        [
            0.155_809_562_382_992_15,
            3.236_155_100_662_467_4,
            -1.422_311_828_335_594_9,
            0.0,
        ],
    ],
    alpha: [
        1.140_099_274_172_029,
        -0.640_133_425_574_345_6,
        0.473_629_653_277_255_9,
        0.026_404_498_125_060_714,
    ],
    beta1: [
        -1.845_346_456_510_443_2,
        2.688_764_531_100_726,
        -0.252_386_650_107_132_3,
        0.408_968_575_516_849_56,
    ],
    beta2: [
        0.496_965_814_158_947_8,
        -0.577_120_286_975_359_2,
        -0.129_197_024_703_222_17,
        0.209_351_497_519_633_6,
    ],
    beta3: [
        2.845_346_456_510_442_5,
        -2.688_764_531_100_725,
        0.252_386_650_107_132_2,
        -0.408_968_575_516_849_45,
    ],
    beta4: [
        0.115_226_638_754_434_33,
        -0.578_770_861_477_38,
        0.285_785_102_816_388_6,
        0.177_759_119_906_557_04,
    ],
    c0: [
        0.0,
        -0.041_992_244_213_164_68,
        0.789_840_546_617_033_3,
        3.750_401_017_156_282_3,
    ],
    c1: [
        0.0,
        0.262_042_820_913_304_66,
        0.058_798_752_320_017_66,
        0.758_661_169_101_175,
    ],
};

/// Tang & Xiao weak-order-2 tableau: upstream `W2ItoCoefficients`.
///
/// ```text
///   | A0 | B0
///   | A1 | B1  B2
/// --+----+----------
///   | α  | β0  β1
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct W2ItoTableau<const S: usize> {
    /// Upstream `alpha`: drift weights.
    pub alpha: [f64; S],
    /// Upstream `beta0`: diffusion weights on the three-point increment.
    pub beta0: [f64; S],
    /// Upstream `beta1`: diffusion weights on the diagonal iterated integral.
    pub beta1: [f64; S],
    /// Upstream `A0`: drift-to-drift stage matrix.
    pub a0: [[f64; S]; S],
    /// Upstream `B0`: diffusion-to-drift stage matrix.
    pub b0: [[f64; S]; S],
    /// Upstream `A1`: drift-to-diffusion stage matrix.
    pub a1: [[f64; S]; S],
    /// Upstream `B1`: same-source diffusion-to-diffusion stage matrix.
    pub b1: [[f64; S]; S],
    /// Upstream `B2`: cross-source diffusion-to-diffusion stage matrix.
    pub b2: [[f64; S]; S],
}

impl<const S: usize> W2ItoTableau<S> {
    /// Drift stage node `c0_i = Σ_j A0_ij` (upstream `c0`).
    pub fn c0(&self, stage: usize) -> f64 {
        self.a0[stage].iter().sum()
    }

    /// Diffusion stage node `c1_i = Σ_j A1_ij` (upstream `c1`).
    pub fn c1(&self, stage: usize) -> f64 {
        self.a1[stage].iter().sum()
    }
}

/// Upstream `svStochasticIntegratorW2Ito1` coefficients (Tang & Xiao 2017,
/// Table 2): three stages, deterministic order 3.
pub const W2ITO1: W2ItoTableau<3> = W2ItoTableau {
    alpha: [1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0],
    beta0: [-1.0, 1.0, 1.0],
    beta1: [2.0, 0.0, -2.0],
    a0: [[0.0, 0.0, 0.0], [1.0 / 2.0, 0.0, 0.0], [-1.0, 2.0, 0.0]],
    b0: [
        [0.0, 0.0, 0.0],
        [(6.0 - SQRT_6) / 10.0, 0.0, 0.0],
        [(3.0 + 2.0 * SQRT_6) / 5.0, 0.0, 0.0],
    ],
    a1: [
        [0.0, 0.0, 0.0],
        [1.0 / 4.0, 0.0, 0.0],
        [1.0 / 4.0, 0.0, 0.0],
    ],
    b1: [
        [0.0, 0.0, 0.0],
        [1.0 / 2.0, 0.0, 0.0],
        [-1.0 / 2.0, 0.0, 0.0],
    ],
    b2: [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
};

/// Upstream `svStochasticIntegratorW2Ito2` coefficients (Tang & Xiao 2017,
/// Table 3): four stages, deterministic order 4.
pub const W2ITO2: W2ItoTableau<4> = W2ItoTableau {
    alpha: [1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0],
    beta0: [0.0, -1.0, 1.0, 1.0],
    beta1: [0.0, 2.0, 0.0, -2.0],
    a0: [
        [0.0, 0.0, 0.0, 0.0],
        [1.0 / 2.0, 0.0, 0.0, 0.0],
        [0.0, 1.0 / 2.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ],
    b0: [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
    ],
    a1: [
        [0.0, 0.0, 0.0, 0.0],
        [1.0 / 2.0, 0.0, 0.0, 0.0],
        [1.0 / 2.0, 0.0, 0.0, 0.0],
        [1.0 / 2.0, 0.0, 0.0, 0.0],
    ],
    b1: [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0 / 2.0, 0.0, 0.0],
        [0.0, -1.0 / 2.0, 0.0, 0.0],
    ],
    b2: [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ],
};

/// Debrabant & Roessler weak-order-2 coefficients shared by DRI1 and the
/// Roessler RI methods: upstream `DRI1Coefficients`, same names.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dri1Coefficients {
    pub a021: f64,
    pub a031: f64,
    pub a032: f64,
    pub a121: f64,
    pub a131: f64,
    pub b021: f64,
    pub b031: f64,
    pub b121: f64,
    pub b131: f64,
    pub b221: f64,
    pub b222: f64,
    pub b223: f64,
    pub b231: f64,
    pub b232: f64,
    pub b233: f64,
    pub alpha1: f64,
    pub alpha2: f64,
    pub alpha3: f64,
    pub c02: f64,
    pub c03: f64,
    pub c12: f64,
    pub c13: f64,
    pub beta11: f64,
    pub beta12: f64,
    pub beta13: f64,
    pub beta22: f64,
    pub beta23: f64,
    pub beta31: f64,
    pub beta32: f64,
    pub beta33: f64,
    pub beta42: f64,
    pub beta43: f64,
}

impl Dri1Coefficients {
    /// Upstream `svStochasticIntegratorDRI1::getCoefficients`.
    pub fn dri1() -> Self {
        Self {
            a021: 1.0 / 2.0,
            a031: -1.0,
            a032: 2.0,
            a121: 342.0 / 491.0,
            a131: 342.0 / 491.0,
            b021: (6.0 - 6.0_f64.sqrt()) / 10.0,
            b031: (3.0 + 2.0 * 6.0_f64.sqrt()) / 5.0,
            b121: 3.0 * (38.0_f64 / 491.0).sqrt(),
            b131: -3.0 * (38.0_f64 / 491.0).sqrt(),
            b221: -214.0 / 513.0 * (1105.0_f64 / 991.0).sqrt(),
            b222: -491.0 / 513.0 * (221.0_f64 / 4955.0).sqrt(),
            b223: -491.0 / 513.0 * (221.0_f64 / 4955.0).sqrt(),
            b231: 214.0 / 513.0 * (1105.0_f64 / 991.0).sqrt(),
            b232: 491.0 / 513.0 * (221.0_f64 / 4955.0).sqrt(),
            b233: 491.0 / 513.0 * (221.0_f64 / 4955.0).sqrt(),
            alpha1: 1.0 / 6.0,
            alpha2: 2.0 / 3.0,
            alpha3: 1.0 / 6.0,
            c02: 1.0 / 2.0,
            c03: 1.0,
            c12: 342.0 / 491.0,
            c13: 342.0 / 491.0,
            beta11: 193.0 / 684.0,
            beta12: 491.0 / 1368.0,
            beta13: 491.0 / 1368.0,
            beta22: 1.0 / 6.0 * (491.0_f64 / 38.0).sqrt(),
            beta23: -1.0 / 6.0 * (491.0_f64 / 38.0).sqrt(),
            beta31: -4955.0 / 7072.0,
            beta32: 4955.0 / 14144.0,
            beta33: 4955.0 / 14144.0,
            beta42: -1.0 / 8.0 * (4955.0_f64 / 221.0).sqrt(),
            beta43: 1.0 / 8.0 * (4955.0_f64 / 221.0).sqrt(),
        }
    }

    /// The cross-term and weight coefficients the four RI tableaus share:
    /// `b221 = 1`, `b231 = -1`, the other `b2..` zero (upstream comment).
    const RI_SHARED: Self = Self {
        a021: 0.0,
        a031: 0.0,
        a032: 0.0,
        a121: 1.0,
        a131: 1.0,
        b021: 0.0,
        b031: 0.0,
        b121: 1.0,
        b131: -1.0,
        b221: 1.0,
        b222: 0.0,
        b223: 0.0,
        b231: -1.0,
        b232: 0.0,
        b233: 0.0,
        alpha1: 0.0,
        alpha2: 0.0,
        alpha3: 0.0,
        c02: 0.0,
        c03: 0.0,
        c12: 1.0,
        c13: 1.0,
        beta11: 1.0 / 2.0,
        beta12: 1.0 / 4.0,
        beta13: 1.0 / 4.0,
        beta22: 1.0 / 2.0,
        beta23: -1.0 / 2.0,
        beta31: -1.0 / 2.0,
        beta32: 1.0 / 4.0,
        beta33: 1.0 / 4.0,
        beta42: 1.0 / 2.0,
        beta43: -1.0 / 2.0,
    };

    /// Upstream `svStochasticIntegratorRI1::getCoefficients` (Roessler 2009).
    pub fn ri1() -> Self {
        Self {
            a021: 2.0 / 3.0,
            a031: -1.0 / 3.0,
            a032: 1.0,
            b021: 1.0,
            b031: 0.0,
            alpha1: 1.0 / 4.0,
            alpha2: 1.0 / 2.0,
            alpha3: 1.0 / 4.0,
            c02: 2.0 / 3.0,
            c03: 2.0 / 3.0,
            ..Self::RI_SHARED
        }
    }

    /// Upstream `svStochasticIntegratorRI3::getCoefficients`.
    pub fn ri3() -> Self {
        Self {
            a021: 1.0,
            a031: 1.0 / 4.0,
            a032: 1.0 / 4.0,
            b021: (3.0 - 2.0 * 6.0_f64.sqrt()) / 5.0,
            b031: (6.0 + 6.0_f64.sqrt()) / 10.0,
            alpha1: 1.0 / 6.0,
            alpha2: 1.0 / 6.0,
            alpha3: 2.0 / 3.0,
            c02: 1.0,
            c03: 1.0 / 2.0,
            ..Self::RI_SHARED
        }
    }

    /// Upstream `svStochasticIntegratorRI5::getCoefficients`.
    pub fn ri5() -> Self {
        Self {
            a021: 1.0,
            a031: 25.0 / 144.0,
            a032: 35.0 / 144.0,
            a121: 1.0 / 4.0,
            a131: 1.0 / 4.0,
            b021: 1.0 / 3.0,
            b031: -5.0 / 6.0,
            b121: 1.0 / 2.0,
            b131: -1.0 / 2.0,
            alpha1: 1.0 / 10.0,
            alpha2: 3.0 / 14.0,
            alpha3: 24.0 / 35.0,
            c02: 1.0,
            c03: 5.0 / 12.0,
            c12: 1.0 / 4.0,
            c13: 1.0 / 4.0,
            beta11: 1.0,
            beta12: -1.0,
            beta13: -1.0,
            beta22: 1.0,
            beta23: -1.0,
            beta31: 1.0 / 2.0,
            beta32: -1.0 / 4.0,
            beta33: -1.0 / 4.0,
            ..Self::RI_SHARED
        }
    }

    /// Upstream `svStochasticIntegratorRI6::getCoefficients`.
    pub fn ri6() -> Self {
        Self {
            a021: 1.0,
            a031: 0.0,
            a032: 0.0,
            b021: 1.0,
            b031: 0.0,
            alpha1: 1.0 / 2.0,
            alpha2: 1.0 / 2.0,
            alpha3: 0.0,
            c02: 1.0,
            c03: 0.0,
            ..Self::RI_SHARED
        }
    }
}

/// Roessler-Stratonovich weak-order-2 coefficients of RS1 and RS2: upstream
/// `RSCoefficients`, same names.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RsCoefficients {
    pub a021: f64,
    pub a031: f64,
    pub a032: f64,
    pub a131: f64,
    pub a141: f64,
    pub b031: f64,
    pub b032: f64,
    pub b121: f64,
    pub b131: f64,
    pub b132: f64,
    pub b141: f64,
    pub b142: f64,
    pub b143: f64,
    pub b221: f64,
    pub b231: f64,
    pub b331: f64,
    pub b332: f64,
    pub b341: f64,
    pub b342: f64,
    pub alpha1: f64,
    pub alpha2: f64,
    pub alpha3: f64,
    pub alpha4: f64,
    pub c02: f64,
    pub c03: f64,
    pub c13: f64,
    pub c14: f64,
    pub beta11: f64,
    pub beta12: f64,
    pub beta13: f64,
    pub beta14: f64,
    pub beta22: f64,
    pub beta23: f64,
}

/// Upstream `svStochasticIntegratorRS1` coefficients (Roessler 2007, Table
/// 5.2): deterministic order 2.
pub const RS1: RsCoefficients = RsCoefficients {
    a021: 0.0,
    a031: 1.0,
    a032: 0.0,
    a131: 1.0,
    a141: 1.0,
    b031: 1.0 / 4.0,
    b032: 3.0 / 4.0,
    b121: 2.0 / 3.0,
    b131: 1.0 / 12.0,
    b132: 1.0 / 4.0,
    b141: -5.0 / 4.0,
    b142: 1.0 / 4.0,
    b143: 2.0,
    b221: 1.0,
    b231: -1.0,
    b331: 1.0 / 4.0,
    b332: 3.0 / 4.0,
    b341: 1.0 / 4.0,
    b342: 3.0 / 4.0,
    alpha1: 0.0,
    alpha2: 0.0,
    alpha3: 1.0 / 2.0,
    alpha4: 1.0 / 2.0,
    c02: 0.0,
    c03: 1.0,
    c13: 1.0,
    c14: 1.0,
    beta11: 1.0 / 8.0,
    beta12: 3.0 / 8.0,
    beta13: 3.0 / 8.0,
    beta14: 1.0 / 8.0,
    beta22: -1.0 / 4.0,
    beta23: 1.0 / 4.0,
};

/// Upstream `svStochasticIntegratorRS2` coefficients (Roessler 2007, Table
/// 5.3): deterministic order 3.
pub const RS2: RsCoefficients = RsCoefficients {
    a021: 2.0 / 3.0,
    a031: 1.0 / 6.0,
    a032: 1.0 / 2.0,
    alpha1: 1.0 / 4.0,
    alpha2: 1.0 / 4.0,
    alpha3: 1.0 / 2.0,
    alpha4: 0.0,
    c02: 2.0 / 3.0,
    c03: 2.0 / 3.0,
    ..RS1
};

/// Tocino & Vigo-Aguiar weak-order-2 coefficients of SIEA, SMEA, SIEB and
/// SMEB: upstream `SIESMECoefficients` (`svIntegratorWeakSIESME.h`), same
/// names.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SieSmeCoefficients {
    /// Drift-stage weight on `k0`.
    pub alpha1: f64,
    /// Drift-stage weight on `k1`.
    pub alpha2: f64,
    /// `g0` noise weight.
    pub gamma1: f64,
    /// `g1` noise weight on `ΔW`.
    pub lambda1: f64,
    /// `g1` noise weight on `√h`.
    pub lambda2: f64,
    /// `g1` noise weight on `W2`.
    pub lambda3: f64,
    /// `g2` noise weight on `ΔW`.
    pub mu1: f64,
    /// `g2` noise weight on `√h`.
    pub mu2: f64,
    /// `g2` noise weight on `W2`.
    pub mu3: f64,
    /// Drift stage node.
    pub mu0: f64,
    /// Diffusion stage node.
    pub mubar0: f64,
    /// `k1` stage drift scaling.
    pub lambda0: f64,
    /// `g1`/`g2` stage drift scaling.
    pub lambdabar0: f64,
    /// `k1` stage noise scaling on `ΔW`.
    pub nu1: f64,
    /// `k1` stage noise scaling on the `ΔW³` term (`W3`).
    pub nu2: f64,
    /// `g1` stage noise scaling on `√h`.
    pub beta2: f64,
    /// `g1` stage noise scaling on `W2`.
    pub beta3: f64,
    /// `g2` stage noise scaling on `√h`.
    pub delta2: f64,
    /// `g2` stage noise scaling on `W2`.
    pub delta3: f64,
}

/// Upstream `svStochasticIntegratorSIEA` coefficients (stochastic improved Euler).
pub const SIEA: SieSmeCoefficients = SieSmeCoefficients {
    alpha1: 0.5,
    alpha2: 0.5,
    gamma1: 0.5,
    lambda1: 0.25,
    lambda2: -0.25,
    lambda3: 0.25,
    mu1: 0.25,
    mu2: 0.25,
    mu3: -0.25,
    mu0: 1.0,
    mubar0: 1.0,
    lambda0: 1.0,
    lambdabar0: 1.0,
    nu1: 1.0,
    nu2: 0.0,
    beta2: 1.0,
    beta3: 0.0,
    delta2: -1.0,
    delta3: 0.0,
};

/// Upstream `svStochasticIntegratorSMEA` coefficients (stochastic modified Euler).
pub const SMEA: SieSmeCoefficients = SieSmeCoefficients {
    alpha1: 0.0,
    alpha2: 1.0,
    mu0: 0.5,
    lambda0: 0.5,
    nu1: (2.0 - SQRT_6) / 4.0,
    nu2: SQRT_6 / 12.0,
    ..SIEA
};

/// Upstream `svStochasticIntegratorSIEB` coefficients.
pub const SIEB: SieSmeCoefficients = SieSmeCoefficients {
    alpha1: 0.5,
    alpha2: 0.5,
    gamma1: -0.2,
    lambda1: 0.6,
    lambda2: 1.5,
    lambda3: -0.5,
    mu1: 0.6,
    mu2: -1.5,
    mu3: 0.5,
    mu0: 1.0,
    mubar0: 5.0 / 12.0,
    lambda0: 1.0,
    lambdabar0: 5.0 / 12.0,
    nu1: 1.0,
    nu2: 0.0,
    beta2: 0.0,
    beta3: -1.0 / 6.0,
    delta2: 0.0,
    delta3: 1.0 / 6.0,
};

/// Upstream `svStochasticIntegratorSMEB` coefficients.
pub const SMEB: SieSmeCoefficients = SieSmeCoefficients {
    alpha1: 0.0,
    alpha2: 1.0,
    mu0: 0.5,
    lambda0: 0.5,
    nu1: (2.0 - SQRT_6) / 4.0,
    nu2: SQRT_6 / 12.0,
    ..SIEB
};

#[cfg(test)]
mod tests {
    //! Port of upstream `GenericTableauxRejectMalformedCoefficients`
    //! (`test_flatStochasticTableaux.cpp`), plus checks that the shipped
    //! tableaus pass the same validation and match their upstream nodes.

    use super::*;

    #[test]
    fn generic_tableaus_reject_malformed_coefficients() {
        let mut sra = SraTableau::<2>::ZERO;
        sra.a0[0][0] = 0.25;
        assert!(sra.validate().unwrap_err().contains("strictly lower"));

        let mut sra = SraTableau::<2>::ZERO;
        sra.beta2[1] = f64::INFINITY;
        assert!(sra.validate().unwrap_err().contains("finite"));

        let mut sri = SriTableau::<2>::ZERO;
        sri.b1[0][1] = -0.5;
        assert!(sri.validate().unwrap_err().contains("strictly lower"));

        let mut sri = SriTableau::<2>::ZERO;
        sri.c1[0] = f64::NAN;
        assert!(sri.validate().unwrap_err().contains("finite"));
    }

    #[test]
    fn shipped_tableaus_are_valid() {
        SRA1.validate().unwrap();
        SOSRA.validate().unwrap();
        SRIW1.validate().unwrap();
        SOSRI.validate().unwrap();
    }

    #[test]
    fn sqrt_six_constant_is_correctly_rounded() {
        assert_eq!(SQRT_6, 6.0_f64.sqrt());
        assert_eq!(W2ITO1.b0[1][0], (6.0 - 6.0_f64.sqrt()) / 10.0);
        assert_eq!(SMEB.nu2, 6.0_f64.sqrt() / 12.0);
        assert_eq!(Dri1Coefficients::ri3().b031, (6.0 + SQRT_6) / 10.0);
    }

    /// Upstream documents the SRI `c0`/`c1` nodes as the row sums of `A0`/`A1`.
    #[test]
    fn sri_nodes_are_row_sums() {
        for tableau in [SRIW1, SOSRI] {
            for stage in 0..4 {
                let c0: f64 = tableau.a0[stage].iter().sum();
                let c1: f64 = tableau.a1[stage].iter().sum();
                assert!((c0 - tableau.c0[stage]).abs() < 1.0e-14);
                assert!((c1 - tableau.c1[stage]).abs() < 1.0e-14);
            }
        }
    }

    #[test]
    fn w2ito_nodes_are_row_sums() {
        assert_eq!(W2ITO1.c0(2), 1.0);
        assert_eq!(W2ITO1.c1(1), 0.25);
        assert_eq!(W2ITO2.c0(3), 1.0);
        assert_eq!(W2ITO2.c1(3), 0.5);
    }
}
