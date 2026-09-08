#!/usr/bin/env python3

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

        normalized_up_vector = compute_up_vector_from_sensor_altitudes(
            np.array(sensor_altitudes)
        )

        quaternion = get_quaternion_from_up_vector(normalized_up_vector, timestamp)

        self.attitude_publisher.publish(quaternion)


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeFromPressureSensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
