package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Robot configuration constants.
 * ロボットの設定定数
 */
public final class RobotConfig {
    private RobotConfig() {} // Prevent instantiation

    // ===================
    // Drive Motors
    // ===================
    public static final class DriveMotor {
        public static final String LEFT_FRONT = "leftFront";
        public static final String RIGHT_FRONT = "rightFront";
        public static final String LEFT_REAR = "leftRear";
        public static final String RIGHT_REAR = "rightRear";

        public static final DcMotor.Direction LEFT_FRONT_DIR = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction RIGHT_FRONT_DIR = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction LEFT_REAR_DIR = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction RIGHT_REAR_DIR = DcMotor.Direction.REVERSE;
    }

    // ===================
    // IMU
    // ===================
    public static final class Imu {
        public static final String NAME = "imu";
        public static final RevHubOrientationOnRobot ORIENTATION =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                );
    }

    // ===================
    // Vision (AprilTag)
    // ===================
    public static final class Vision {
        public static final String CAMERA_NAME = "Webcam 1";
        // AprilTag field positions will be added here after Road Runner integration
    }

    // ===================
    // Control Parameters
    // ===================
    public static final class Control {
        public static final double DEAD_ZONE = 0.1;
        public static final double STRAFE_CORRECTION = 1.1;
    }
}