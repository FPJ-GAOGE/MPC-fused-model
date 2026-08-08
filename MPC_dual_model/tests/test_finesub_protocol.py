import unittest
from dataclasses import replace

import numpy as np

from MPC_dual_model.finesub_protocol import (
    COMMAND_STATUS_ACCEPTED,
    COMMAND_FRAME_SIZE,
    TELEMETRY_FRAME_SIZE,
    FineSUBHardwareAdapter,
    FineSUBTelemetry,
    TelemetryStreamDecoder,
    crc16_modbus,
    is_newer_telemetry,
    is_newer_u16_sequence,
    motor_throttles_to_channels,
    pack_command,
    pack_telemetry,
    unpack_command,
    unpack_command_frame,
    unpack_telemetry,
)


class FineSUBProtocolTest(unittest.TestCase):
    def test_crc_matches_modbus_reference_vector(self) -> None:
        self.assertEqual(crc16_modbus(b"123456789"), 0x4B37)

    def test_mpc_wrench_round_trips_through_normalized_channels(self) -> None:
        adapter = FineSUBHardwareAdapter()
        command = adapter.convert((10.0, -7.5, 3.0), 2.0, armed=True)
        frame = pack_command(
            command,
            0x1234,
            session_id=0x10203040,
            sender_time_ms=123456,
        )
        self.assertEqual(len(frame), COMMAND_FRAME_SIZE)
        decoded, sequence = unpack_command(frame)
        self.assertEqual(sequence, 0x1234)
        self.assertTrue(decoded.armed)
        self.assertTrue(decoded.yaw_direct)
        envelope = unpack_command_frame(frame)
        self.assertEqual(envelope.session_id, 0x10203040)
        self.assertEqual(envelope.sender_time_ms, 123456)

        telemetry = FineSUBTelemetry(
            sequence=sequence,
            tick_ms=100,
            state=2,
            armed=True,
            mpc_direct=True,
            yaw_direct=True,
            failsafe=False,
            yaw_rad=0.20,
            yaw_rate_rad_s=-0.05,
            depth_m=0.8,
            forward=decoded.forward,
            right=decoded.right,
            down=decoded.down,
            yaw=decoded.yaw,
        )
        force, moment = adapter.achieved_wrench(telemetry)
        np.testing.assert_allclose(force, (10.0, -7.5, 3.0), atol=1.0e-6)
        self.assertAlmostEqual(moment, 2.0, places=6)

    def test_stream_decoder_handles_noise_fragmentation_and_bad_crc(self) -> None:
        telemetry = FineSUBTelemetry(
            sequence=7,
            tick_ms=1234,
            state=2,
            armed=True,
            mpc_direct=True,
            yaw_direct=False,
            failsafe=False,
            yaw_rad=0.35,
            yaw_rate_rad_s=0.07,
            depth_m=1.2,
            forward=0.1,
            right=-0.2,
            down=0.3,
            yaw=-0.05,
        )
        frame = pack_telemetry(telemetry)
        self.assertEqual(len(frame), TELEMETRY_FRAME_SIZE)
        bad = bytearray(frame)
        bad[-1] ^= 0xFF

        decoder = TelemetryStreamDecoder()
        self.assertEqual(decoder.feed(b"noise" + bytes(bad) + frame[:9]), [])
        output = decoder.feed(frame[9:] + frame)
        self.assertEqual([item.sequence for item in output], [7, 7])
        self.assertFalse(output[0].yaw_direct)
        self.assertAlmostEqual(output[0].depth_m, 1.2, places=6)
        # The resynchronizer counts both the leading noise and bytes skipped
        # while scanning past the deliberately corrupted frame.
        self.assertGreaterEqual(decoder.dropped_bytes, 5)
        self.assertEqual(decoder.crc_errors, 1)

    def test_full_v3_telemetry_round_trip(self) -> None:
        telemetry = FineSUBTelemetry(
            sequence=17,
            tick_ms=9988,
            state=2,
            armed=True,
            mpc_direct=True,
            yaw_direct=True,
            failsafe=False,
            yaw_rad=0.4,
            yaw_rate_rad_s=-0.2,
            depth_m=1.3,
            forward=0.1,
            right=-0.2,
            down=0.3,
            yaw=-0.04,
            command_status=COMMAND_STATUS_ACCEPTED,
            last_command_session=0xAABBCCDD,
            last_command_sequence=9,
            last_command_crc=0x1234,
            last_command_sender_time_ms=55,
            command_count=19,
            rejected_command_count=2,
            quat_wxyz=(0.9, 0.1, 0.2, 0.3),
            angular_velocity_xyz=(0.01, 0.02, -0.2),
            linear_acceleration_xyz=(1.0, 2.0, 3.0),
            pressure_pa=12345.0,
            applied_motor_throttle=tuple(i / 20.0 for i in range(8)),
            motor_rpm=tuple(100.0 * i for i in range(8)),
            execution_feedback_valid=True,
            rpm_available=True,
            rpm_valid_mask=0xA5,
        )
        decoded = unpack_telemetry(pack_telemetry(telemetry))
        self.assertTrue(decoded.last_command_accepted)
        self.assertEqual(decoded.last_command_session, 0xAABBCCDD)
        self.assertEqual(decoded.last_command_crc, 0x1234)
        self.assertTrue(decoded.execution_feedback_valid)
        self.assertTrue(decoded.rpm_available)
        self.assertEqual(decoded.rpm_valid_mask, 0xA5)
        np.testing.assert_allclose(decoded.quat_wxyz, telemetry.quat_wxyz)
        np.testing.assert_allclose(decoded.motor_rpm, telemetry.motor_rpm)

    def test_applied_motor_feedback_is_inverted_into_achieved_wrench(self) -> None:
        translation = np.array([0.12, -0.08, 0.25])
        yaw = 0.06
        roll_pitch = np.array([0.03, -0.04])
        lower_mixer = np.array(
            [
                [1.0, -1.0, 1.0],
                [-1.0, 1.0, 1.0],
                [1.0, 1.0, -1.0],
                [-1.0, -1.0, -1.0],
            ]
        )
        upper_mixer = np.array(
            [
                [1.0, -1.0, -1.0],
                [1.0, 1.0, -1.0],
                [-1.0, 1.0, -1.0],
                [-1.0, -1.0, -1.0],
            ]
        )
        lower = lower_mixer @ np.array([yaw, translation[0], translation[1]])
        upper = upper_mixer @ np.array([roll_pitch[0], roll_pitch[1], translation[2]])
        physical = np.array(
            [lower[0], upper[0], -upper[1], -lower[1], lower[2], upper[2], upper[3], -lower[3]]
        )
        reconstructed, reconstructed_yaw, reconstructed_roll_pitch = (
            motor_throttles_to_channels(physical)
        )
        np.testing.assert_allclose(reconstructed, translation, atol=1e-12)
        self.assertAlmostEqual(reconstructed_yaw, yaw)
        np.testing.assert_allclose(reconstructed_roll_pitch, roll_pitch, atol=1e-12)

        adapter = FineSUBHardwareAdapter()
        telemetry = FineSUBTelemetry(
            sequence=1,
            tick_ms=2,
            state=2,
            armed=True,
            mpc_direct=True,
            yaw_direct=True,
            failsafe=False,
            yaw_rad=0.0,
            yaw_rate_rad_s=0.0,
            depth_m=0.0,
            forward=0.0,
            right=0.0,
            down=0.0,
            yaw=0.0,
            applied_motor_throttle=tuple(physical),
            execution_feedback_valid=True,
        )
        force, moment = adapter.achieved_wrench(telemetry)
        expected_force, expected_moment = adapter._channels_to_wrench(
            translation, yaw
        )
        np.testing.assert_allclose(force, expected_force, atol=1e-6)
        self.assertAlmostEqual(moment, expected_moment, places=6)

    def test_sequence_order_handles_duplicates_old_frames_and_wrap(self) -> None:
        self.assertTrue(is_newer_u16_sequence(11, 10))
        self.assertFalse(is_newer_u16_sequence(10, 10))
        self.assertFalse(is_newer_u16_sequence(9, 10))
        self.assertTrue(is_newer_u16_sequence(0, 0xFFFF))
        self.assertFalse(is_newer_u16_sequence(0xFFFF, 0))

        previous = FineSUBTelemetry(
            sequence=100,
            tick_ms=5000,
            state=2,
            armed=True,
            mpc_direct=True,
            yaw_direct=True,
            failsafe=False,
            yaw_rad=0.0,
            yaw_rate_rad_s=0.0,
            depth_m=0.5,
            forward=0.0,
            right=0.0,
            down=0.0,
            yaw=0.0,
        )
        self.assertFalse(is_newer_telemetry(replace(previous), previous))
        self.assertFalse(
            is_newer_telemetry(replace(previous, sequence=99), previous)
        )
        self.assertTrue(
            is_newer_telemetry(
                replace(
                    previous,
                    sequence=0,
                    tick_ms=20,
                    state=0,
                    armed=False,
                ),
                previous,
            )
        )

    def test_disarmed_or_failsafe_telemetry_maps_to_zero_wrench(self) -> None:
        adapter = FineSUBHardwareAdapter()
        active = FineSUBTelemetry(
            sequence=1,
            tick_ms=50,
            state=2,
            armed=True,
            mpc_direct=True,
            yaw_direct=True,
            failsafe=False,
            yaw_rad=0.0,
            yaw_rate_rad_s=0.0,
            depth_m=0.5,
            forward=0.2,
            right=-0.1,
            down=0.3,
            yaw=0.1,
        )
        for telemetry in (
            replace(active, armed=False),
            replace(active, failsafe=True),
            replace(active, mpc_direct=False),
        ):
            with self.subTest(telemetry=telemetry):
                force, moment = adapter.achieved_wrench(telemetry)
                np.testing.assert_array_equal(force, np.zeros(3))
                self.assertEqual(moment, 0.0)

    def test_nonfinite_telemetry_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            FineSUBTelemetry(
                sequence=1,
                tick_ms=50,
                state=2,
                armed=True,
                mpc_direct=True,
                yaw_direct=True,
                failsafe=False,
                yaw_rad=float("nan"),
                yaw_rate_rad_s=0.0,
                depth_m=0.5,
                forward=0.0,
                right=0.0,
                down=0.0,
                yaw=0.0,
            )


if __name__ == "__main__":
    unittest.main()
