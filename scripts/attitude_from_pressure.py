#!/usr/bin/env python3

import collections

import rclpy
from rclpy.node import Node

import message_filters
from geometry_msgs.msg import QuaternionStamped
from rclpy.time import Time

from rtf_sensors_msgs.msg import CustomPressureTemperature

# numpy's bundled OpenBLAS starts a worker pool that busy-spins (via sched_yield)
# between calls. Every matrix here is 3x3 or 4x4, so threading buys nothing and the
# spinning costs a full core per worker. Must be set before numpy is imported.
import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np
import pandas as pd
from scipy.signal import butter, lfilter, lfilter_zi


def load_c_from_csv(csv_file_path: str) -> dict[int, np.ndarray]:
    data = pd.read_csv(csv_file_path)

    matrices = {}

    for sensor_id, sensor_data in data.groupby("sensor_id"):
        C = np.zeros((4, 4), dtype=float)

        for _, row in sensor_data.iterrows():
            i = int(row["i_power"])
            j = int(row["j_power"])
            C[i, j] = float(row["coefficient"])

        matrices[sensor_id] = C

    return matrices


def apply_temperature_compensation(
    raw_pressure: float,
    scale_pressure: float,
    raw_temperature: float,
    scale_temperature: float,
    C: np.ndarray,
) -> float:
    scaled_temperature = raw_temperature / scale_temperature
    scaled_pressure = raw_pressure / scale_pressure

    pressure_vec = np.array(
        [1.0, scaled_pressure, scaled_pressure**2, scaled_pressure**3]
    )
    temperature_vec = np.array(
        [1.0, scaled_temperature, scaled_temperature**2, scaled_temperature**3]
    ).reshape(-1, 1)

    compensated_pressure = float(pressure_vec @ C @ temperature_vec)

    return compensated_pressure


def compute_altitude_hypsometric(
    sensor_temperature_kelvin: float, sensor_pressure: float, reference_pressure: float
) -> float:
    g = 9.80665
    Rd = 287.0

    altitude = (
        Rd
        * (sensor_temperature_kelvin)
        / g
        * np.log(reference_pressure / sensor_pressure)
    )

    return altitude


SENSOR_POSITIONS_X_Y_Z = {
    "dps310_0": np.array([0.0, 0.0, 0.0]),
    "dps310_1": np.array([0.0325, 0.0, 0.0]),
    "dps310_2": np.array([1.294119, -0.172, -0.002]),
    "dps310_3": np.array([1.294119, -0.172, -0.058]),
    "dps310_4": np.array([-0.028, 1.125302, -0.002]),
    "dps310_5": np.array([-0.028, 1.125302, -0.058]),
    "dps310_6": np.array([-0.056, -0.172, 1.2925]),
    "dps310_7": np.array([0.0, -0.172, 1.2925]),
    "setra": np.array([0.075, -0.025, 0.0]),
}


def _build_sensor_position_matrix() -> np.ndarray:
    """Positions of the sensor pairs relative to the origin (avg of 0 and 1)."""
    pair_centers = [
        (SENSOR_POSITIONS_X_Y_Z[f"dps310_{a}"] + SENSOR_POSITIONS_X_Y_Z[f"dps310_{b}"])
        / 2
        for a, b in ((0, 1), (2, 3), (4, 5), (6, 7))
    ]
    origin = pair_centers[0]

    return np.array([center - origin for center in pair_centers[1:]])


# The geometry is fixed, so invert it once at import instead of on every message.
P = _build_sensor_position_matrix()
P_INV = np.linalg.inv(P)


def compute_up_vector_from_sensor_altitudes(
    sensor_altitudes: np.ndarray,
) -> np.ndarray:
    # Compute the height differences based on altitude readings
    pair_altitudes = (sensor_altitudes[0::2] + sensor_altitudes[1::2]) / 2
    heights = pair_altitudes[1:] - pair_altitudes[0]

    up_vector = P_INV @ heights

    return up_vector / np.linalg.norm(up_vector)


def get_quaternion_from_up_vector(
    up_vector: np.ndarray, timestamp: Time
) -> QuaternionStamped:
    q_w = np.sqrt((up_vector[2] + 1) / 2)
    q_x = up_vector[1] / (2 * q_w)
    q_y = -up_vector[0] / (2 * q_w)
    q_z = 0.0

    quat = QuaternionStamped()
    quat.header.stamp = timestamp
    quat.quaternion.x = q_x
    quat.quaternion.y = q_y
    quat.quaternion.z = q_z
    quat.quaternion.w = q_w

    return quat


# The offline sweep that picked these defaults used centered, reflect-padded smoothing
# and filtfilt, both of which read future samples. A node feeding the mapper's
# odom -> base_link TF cannot: a late attitude is a failed lookupTransform and a scan
# dropped from the map. So these are the causal equivalents, and 31 samples / 1.0 Hz are
# starting points to re-sweep on the rig rather than transplanted optima.


class MovingAverageFilter:
    """Trailing boxcar over the last `window_samples` vectors."""

    def __init__(self, window_samples: int):
        if window_samples < 1:
            raise ValueError(f"filter_window_samples must be >= 1, got {window_samples}")
        self.window_samples = window_samples
        self.buffer = collections.deque(maxlen=window_samples)

    def __call__(self, vector: np.ndarray) -> np.ndarray:
        # A partial average while the window fills, so the attitude stream never has a
        # gap at startup.
        self.buffer.append(vector)
        return np.mean(self.buffer, axis=0)

    def describe(self) -> str:
        return f"moving_average(window_samples={self.window_samples})"


class LowPassFilter:
    """Single-pass Butterworth, one independent filter state per axis."""

    def __init__(self, cutoff_hz: float, order: int, sample_rate_hz: float):
        nyquist = sample_rate_hz / 2.0
        if not 0.0 < cutoff_hz < nyquist:
            raise ValueError(
                f"filter_cutoff_hz must be in (0, {nyquist}), got {cutoff_hz}"
            )

        self.cutoff_hz = cutoff_hz
        self.order = order
        self.sample_rate_hz = sample_rate_hz
        self.b, self.a = butter(order, cutoff_hz / nyquist, btype="low")
        self.state = None

    def __call__(self, vector: np.ndarray) -> np.ndarray:
        if self.state is None:
            # Steady-state response to a step of the first sample, so the filter starts
            # settled on the current attitude instead of ramping up from zero.
            self.state = np.outer(lfilter_zi(self.b, self.a), vector)

        filtered = np.empty(3)
        for axis in range(3):
            output, self.state[:, axis] = lfilter(
                self.b, self.a, vector[axis : axis + 1], zi=self.state[:, axis]
            )
            filtered[axis] = output[0]

        return filtered

    def describe(self) -> str:
        return (
            f"lowpass(cutoff_hz={self.cutoff_hz}, order={self.order}, "
            f"sample_rate_hz={self.sample_rate_hz})"
        )


NUMBER_OF_SENSORS = 8


class AttitudeFromPressureSensors(Node):
    def __init__(self):
        super().__init__("attitude_from_pressure_sensors")

        self.declare_parameter("temperature_calibration_csv_path", "")
        temperature_calibration_csv_path: str = (
            self.get_parameter("temperature_calibration_csv_path")
            .get_parameter_value()
            .string_value
        )
        if not temperature_calibration_csv_path:
            self.get_logger().error(
                "Parameter 'temperature_calibration_csv_path' is empty or not set."
            )
            raise RuntimeError(
                "Missing required parameter 'temperature_calibration_csv_path'"
            )

        self.C_matrices = load_c_from_csv(temperature_calibration_csv_path)

        self.attitude_filter = self._build_filter()
        self.non_finite_count = 0
        self.last_up_vector = None

        self.pressure_subs = []
        for i in range(NUMBER_OF_SENSORS):
            topic = f"dps310_{i}/data"
            self.pressure_subs.append(
                message_filters.Subscriber(self, CustomPressureTemperature, topic)
            )
            self.get_logger().info(
                f"Subscribed to pressure sensor {i} on topic '{topic}'"
            )

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.pressure_subs,
            queue_size=10,
            slop=1.0 / 45.0,
        )
        self.synchronizer.registerCallback(self.synced_pressure_callback)

        self.attitude_publisher = self.create_publisher(
            QuaternionStamped, "attitude_topic", 10
        )

        # With a filter on, the unfiltered attitude goes out alongside it so a single
        # replay yields both .tum files over identical data.
        self.raw_attitude_publisher = None
        if self.attitude_filter is not None:
            self.raw_attitude_publisher = self.create_publisher(
                QuaternionStamped, "attitude_topic_raw", 10
            )

    def _build_filter(self):
        """Resolve the `data_filter` parameter into a callable, or None."""
        self.declare_parameter("data_filter", "none")
        self.declare_parameter("filter_window_samples", 31)
        self.declare_parameter("filter_cutoff_hz", 1.0)
        self.declare_parameter("filter_order", 2)
        self.declare_parameter("sample_rate_hz", 45.0)

        data_filter = self.get_parameter("data_filter").value

        if data_filter == "none":
            attitude_filter = None
        elif data_filter == "moving_average":
            attitude_filter = MovingAverageFilter(
                window_samples=self.get_parameter("filter_window_samples").value
            )
        elif data_filter == "lowpass":
            attitude_filter = LowPassFilter(
                cutoff_hz=self.get_parameter("filter_cutoff_hz").value,
                order=self.get_parameter("filter_order").value,
                sample_rate_hz=self.get_parameter("sample_rate_hz").value,
            )
        else:
            raise ValueError(
                f"data_filter must be one of 'none', 'moving_average', 'lowpass', "
                f"got '{data_filter}'"
            )

        # Logged so run.log records which filter produced a given results folder: the
        # parameters live in the launch file, which run_manifest.py does not copy.
        description = "none" if attitude_filter is None else attitude_filter.describe()
        self.get_logger().info(f"Attitude filter: {description}")

        return attitude_filter

    def synced_pressure_callback(self, *msgs: CustomPressureTemperature):
        timestamp = msgs[0].header.stamp

        # reference from sensor 0
        reference_pressure = apply_temperature_compensation(
            raw_pressure=msgs[0].raw_pressure,
            scale_pressure=msgs[0].scale_pressure,
            raw_temperature=msgs[0].raw_temperature,
            scale_temperature=msgs[0].scale_temperature,
            C=self.C_matrices[0],
        )

        sensor_altitudes = []
        for i in range(NUMBER_OF_SENSORS):
            msg = msgs[i]

            C = self.C_matrices[i]

            compensated_pressure = apply_temperature_compensation(
                raw_pressure=msg.raw_pressure,
                scale_pressure=msg.scale_pressure,
                raw_temperature=msg.raw_temperature,
                scale_temperature=msg.scale_temperature,
                C=C,
            )
            temperature_kelvin = msg.temperature + 273.15

            altitude_sensor = compute_altitude_hypsometric(
                temperature_kelvin, compensated_pressure, reference_pressure
            )

            sensor_altitudes.append(altitude_sensor)

        up_vector = compute_up_vector_from_sensor_altitudes(np.array(sensor_altitudes))

        if not np.all(np.isfinite(up_vector)):
            # A non-finite pressure poisons an IIR's state permanently, so the sample
            # never reaches the filter. Holding the last output keeps the topic alive at
            # rate: a gap here means no odom -> base_link and scans that fail to register.
            self.non_finite_count += 1
            self.get_logger().warn(
                f"Non-finite up vector, holding last attitude "
                f"({self.non_finite_count} dropped so far)",
                throttle_duration_sec=5.0,
            )
            if self.last_up_vector is None:
                return
            filtered_up_vector = self.last_up_vector
        else:
            if self.raw_attitude_publisher is not None:
                self.raw_attitude_publisher.publish(
                    get_quaternion_from_up_vector(up_vector, timestamp)
                )

            if self.attitude_filter is None:
                filtered_up_vector = up_vector
            else:
                # Filtering a unit vector keeps every sample weighted equally. The raw
                # vector's magnitude is the height-difference scale, which drifts with
                # temperature and would drag the direction with it.
                filtered = self.attitude_filter(up_vector)
                filtered_up_vector = filtered / np.linalg.norm(filtered)

            self.last_up_vector = filtered_up_vector

        quaternion = get_quaternion_from_up_vector(filtered_up_vector, timestamp)

        self.attitude_publisher.publish(quaternion)


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeFromPressureSensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
