package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Mecanum drive subsystem with field-centric control and localization support.
 *
 * フィールドセントリック制御と自己位置推定をサポートするメカナムドライブサブシステム。
 */
public class Drive {
    // Motors
    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;

    // IMU
    private final IMU imu;

    // Vision (for AprilTag localization)
    private Vision vision;

    // Current pose estimate
    private Pose2d pose = new Pose2d();

    public Drive(HardwareMap hardwareMap) {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, RobotConfig.DriveMotor.LEFT_FRONT);
        leftFront.setDirection(RobotConfig.DriveMotor.LEFT_FRONT_DIR);

        rightFront = hardwareMap.get(DcMotor.class, RobotConfig.DriveMotor.RIGHT_FRONT);
        rightFront.setDirection(RobotConfig.DriveMotor.RIGHT_FRONT_DIR);

        leftRear = hardwareMap.get(DcMotor.class, RobotConfig.DriveMotor.LEFT_REAR);
        leftRear.setDirection(RobotConfig.DriveMotor.LEFT_REAR_DIR);

        rightRear = hardwareMap.get(DcMotor.class, RobotConfig.DriveMotor.RIGHT_REAR);
        rightRear.setDirection(RobotConfig.DriveMotor.RIGHT_REAR_DIR);

        // Set brake mode
        leftFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, RobotConfig.Imu.NAME);
        imu.initialize(new IMU.Parameters(RobotConfig.Imu.ORIENTATION));
        imu.resetYaw();
    }

    /**
     * Set vision subsystem for AprilTag localization.
     * AprilTag位置推定用のVisionサブシステムを設定する。
     */
    public void setVision(Vision vision) {
        this.vision = vision;
    }

    /**
     * Get current heading from IMU.
     * IMUから現在の方位を取得する。
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Reset IMU yaw.
     * IMUのヨーをリセットする。
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    /**
     * Get current pose estimate.
     * 現在の位置推定を取得する。
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Set pose estimate.
     * 位置推定を設定する。
     */
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * Update localization. Call this every loop.
     * This will be enhanced with TwoDeadWheel + AprilTag after Road Runner integration.
     *
     * 自己位置推定を更新する。毎ループ呼び出すこと。
     * Road Runner導入後、TwoDeadWheel + AprilTagで強化される。
     */
    public void updateLocalization() {
        // For now, just update heading from IMU
        // After Road Runner integration, this will include:
        // 1. TwoDeadWheel odometry update
        // 2. AprilTag correction (if visible)
        double heading = getHeading();
        pose = new Pose2d(pose.x, pose.y, heading);

        // TODO: Add AprilTag correction when vision is available
        // if (vision != null) {
        //     correctPoseWithAprilTag();
        // }
    }

    // ===================
    // TeleOp: Direct control
    // ===================

    /**
     * Drive with field-centric control.
     * フィールドセントリック制御でドライブする。
     *
     * @param xSpeed   Strafe speed (-1 to 1, positive = right)
     * @param ySpeed   Forward speed (-1 to 1, positive = forward)
     * @param rotation Rotation speed (-1 to 1, positive = counter-clockwise)
     */
    public void drive(double xSpeed, double ySpeed, double rotation) {
        double heading = getHeading();

        // Convert to field-centric
        double rotX = xSpeed * Math.cos(-heading) - ySpeed * Math.sin(-heading);
        double rotY = xSpeed * Math.sin(-heading) + ySpeed * Math.cos(-heading);

        // Apply strafe correction
        rotX *= RobotConfig.Control.STRAFE_CORRECTION;

        // Normalize motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1.0);

        // Calculate motor powers
        double leftFrontPower = (rotY + rotX + rotation) / denominator;
        double leftRearPower = (rotY - rotX + rotation) / denominator;
        double rightFrontPower = (rotY - rotX - rotation) / denominator;
        double rightRearPower = (rotY + rotX - rotation) / denominator;

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     * Drive with robot-centric control.
     * ロボットセントリック制御でドライブする。
     */
    public void driveRobotCentric(double xSpeed, double ySpeed, double rotation) {
        double rotX = xSpeed * RobotConfig.Control.STRAFE_CORRECTION;

        double denominator = Math.max(Math.abs(ySpeed) + Math.abs(rotX) + Math.abs(rotation), 1.0);

        double leftFrontPower = (ySpeed + rotX + rotation) / denominator;
        double leftRearPower = (ySpeed - rotX + rotation) / denominator;
        double rightFrontPower = (ySpeed - rotX - rotation) / denominator;
        double rightRearPower = (ySpeed + rotX - rotation) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     * Stop all motors.
     * 全モーターを停止する。
     */
    public void stop() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    // ===================
    // Autonomous: Actions (placeholder for Road Runner integration)
    // ===================

    // TODO: Add after Road Runner integration:
    // public Action driveTo(Pose2d target) { ... }
    // public Action followTrajectory(TrajectoryAction trajectory) { ... }
    // public Action alignToTag(int tagId, Pose2d targetOffset) { ... }
    // public Action correctPoseWithTag(int tagId, Pose2d tagFieldPosition) { ... }
}
