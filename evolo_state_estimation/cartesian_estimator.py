import math

import numpy as np
from stonesoup.models.measurement.nonlinear import Cartesian2DToBearing
from stonesoup.models.transition.linear import (
    CombinedLinearGaussianTransitionModel, ConstantVelocity)
from stonesoup.predictor.kalman import KalmanPredictor
from stonesoup.types.angle import Bearing
from stonesoup.types.detection import Detection
from stonesoup.types.hypothesis import SingleHypothesis
from stonesoup.types.state import GaussianState
from stonesoup.updater.kalman import ExtendedKalmanUpdater


class CartesianEKF:
    def __init__(
        self,
        initial_range,
        initial_range_std,
        initial_velocity_std,
        bearing_std,  # Degrees.
        acceleration,  # Noise PSD, m²/s³.
        updater_type=ExtendedKalmanUpdater,
    ):
        self.initial_range = initial_range
        self.range_std = initial_range_std
        self.velocity_std = initial_velocity_std
        self.bearing_std = math.radians(bearing_std)
        self.state = None

        motion_model = CombinedLinearGaussianTransitionModel([
            ConstantVelocity(acceleration),
            ConstantVelocity(acceleration),
        ])  # TODO: check arguments
        self.predictor = KalmanPredictor(motion_model)
        self.updater = updater_type()

    def step(self, timestamp, bearing, origin):
        if self.state is not None and timestamp <= self.state.timestamp:
            raise ValueError("Observation time must increase")

        # Initialize if no previous state present
        if self.state is None:
            r = self.initial_range
            c_beta = math.cos(bearing)
            s_beta = math.sin(bearing)

            x = origin[0] + c_beta * r
            y = origin[1] + s_beta * r

            # Diagonal approximation in state order [x, vx, y, vy].
            P_0 = np.diag([
                self.range_std**2, self.velocity_std**2,
                self.range_std**2, self.velocity_std**2,
            ])
            self.state = GaussianState(
                [x, 0, y, 0],
                P_0,
                timestamp=timestamp,
            )
        else:
            prediction = self.predictor.predict(
                self.state, timestamp=timestamp,
            )
            model = Cartesian2DToBearing(
                ndim_state=4,
                mapping=(0, 2),
                noise_covar=np.array([[self.bearing_std**2]]),
                translation_offset=np.asarray(origin).reshape(2, 1),
            )
            detection = Detection(
                [Bearing(bearing)],
                timestamp=timestamp,
                measurement_model=model,
            )
            self.state = self.updater.update(
                SingleHypothesis(prediction, detection),
            )
            prediction.prior = None
  
        # Stone Soup [x, vx, y, vy] → ROS [x, y, vx, vy].
        order = [0, 2, 1, 3]
        return (
            np.asarray(self.state.state_vector[order, 0]).reshape(4).copy(),
            np.asarray(self.state.covar[np.ix_(order, order)]).copy(),
        )
