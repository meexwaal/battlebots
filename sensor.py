import numpy as np
from dataclasses import dataclass

@dataclass
class Sensor:
    "Model a sensor, including fixed sampling rate and noise effects"
    # TODO: temperature effects?

    sample_rate: float

    # Clamp sensed value to this range.
    # Also used for the curve of non-linear effects
    min_val: float
    max_val: float

    # Quantize reading. TODO
    # bits: int = None

    #######
    # Noise
    # These effects cannot be calibrated out.

    # Noise can be expressed as a standard deviation, or more commonly in
    # datasheets as a noise spectrum density, in <units> per root Hz. Sample
    # rate is used as the bandwidth to convert spectrum density into std dev.
    noise_std: float = None
    noise_density: float = None # units/sqrt(Hz)

    ########
    # Biases
    # These effects can be calibrated out, though they may vary with state such
    # as temperature.

    # Constant offset of the value. bias > 0 => sensed > truth.
    bias: float = 0

    # Offset proportional to the true value. bias > 0 => sensed > truth.
    # e.g. a value of 1 would make the sensed value double the true value.
    proportional_bias: float = 0

    # The sensed signal has some relationship with the true value which may be
    # non-linear. It may be approximated as a line of best fit, which leaves
    # some error in parts of the curve. (As modeled, the mean error will be 0;
    # use a bias to add non-zero mean error.) This measures the maximum error as
    # a fraction of the full scale (max - min).
    nonlinearity: float = 0

    def __post_init__(self):
        self.full_scale = self.max_val - self.min_val

        if self.noise_density is not None:
            assert self.noise_std is None, 'Only one noise figure may be specified'
            # TODO model flicker noise?
            self.noise_std = self.noise_density * np.sqrt(self.sample_rate)

        if self.noise_std is None:
            # Default to no noise
            self.noise_std = 0

        # Only update at the sample rate.
        # step() will set .sense and .updated
        self.sample_dt = 1 / self.sample_rate
        self.time = 0
        # TODO: this is kind of janky, how do we make it regular for various update rates?
        self.prev_sample_time = -self.sample_dt

        # Latest simulated value
        self.sense = None
        self.updated = False

    def step(self, truth, dt):
        """
        Step the sensor.
        truth: True value of measurement.
        dt: Time step (s).

        If it's not time to take a new measurement, set .updated to False and return None.
        Otherwise, model the sensed value. Set .sense to this value and return it, and
        set .updated to True.
        """

        # Only update at the sample rate
        self.time += dt
        if (self.time - self.prev_sample_time) < self.sample_dt:
            self.updated = False
            return None
        self.prev_sample_time += self.sample_dt
        self.updated = True

        mean = (
            truth +
            self.bias +
            self.proportional_bias * truth +
            self.calc_nonlinear_bias(truth))
        std = self.noise_std
        val = np.random.normal(mean, std)
        self.sense = np.clip(val, self.min_val, self.max_val)
        return self.sense

    def calc_nonlinear_bias(self, val):
        if self.nonlinearity == 0:
            return 0

        # Model all nonlinearity as the same curve: (x^3 + kx) / (1 + k)
        # https://www.desmos.com/calculator/yskqfbfccu
        # Range and domain are [-1, 1]
        #
        # Error is (x^3 - x) / (1 + k), which has extrema at +/- 3^(-1/2)
        # Extrema have abs value (3^(-3/2) - 3^(-1/2)) / (1 + k) = 0.385 / (1+k)
        # On this curve, full scale is 2, so:
        #   abs(extrema) = 2 * nonlinearity fraction
        #   0.385 / (1+k) = 2 * self.nonlinearity
        k = 0.1925 / self.nonlinearity - 1

        # Map input to [-1, 1]
        x = (val - self.min_val) / self.full_scale * 2 - 1

        bias = (x**3 - x) / (1 + k)

        # Map to scale of output domain
        return bias / 2 * self.full_scale
