package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Drive subsystem wrapping Road Runner's MecanumDrive.
 * Provides both TeleOp direct control and Autonomous trajectory following.
 *
 * Road RunnerのMecanumDriveをラップするドライブサブシステム。
 * TeleOpの直接制御とAutonomousの軌道追従の両方を提供。
 */
public class Drive {
    // Road Runner MecanumDrive
    private final MecanumDrive mecanumDrive;

    // Vision (for AprilTag localization)
    private Vision vision;

    public Drive(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d(0, 0, 0));
    }

    public Drive(HardwareMap hardwareMap, Pose2d initialPose) {
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
    }

    /**
     * Set vision subsystem for AprilTag localization.
     * AprilTag位置推定用のVisionサブシステムを設定する。
     */
    public void setVision(Vision vision) {
        this.vision = vision;
    }

    // ===================
    // Pose Management
    // ===================

    /**
     * Get current pose estimate.
     * 現在の位置推定を取得する。
     */
    public Pose2d getPose() {
        return mecanumDrive.localizer.getPose();
    }

    /**
     * Set pose estimate.
     * 位置推定を設定する。
     */
    public void setPose(Pose2d pose) {
        mecanumDrive.localizer.setPose(pose);
    }

    /**
     * Get current heading in radians.
     * 現在の方位をラジアンで取得する。
     */
    public double getHeading() {
        return getPose().heading.toDouble();
    }

    /**
     * Update localization. Call this every loop.
     * 自己位置推定を更新する。毎ループ呼び出すこと。
     */
    public void updateLocalization() {
        mecanumDrive.updatePoseEstimate();

        // AprilTag correction if vision is available
        // デバッグ用: 一時的に無効化
        // TODO: 問題解決後にコメントを外す
        /*
        if (vision != null) {
            Pose2d visionPose = vision.getBestPoseEstimate();
            if (visionPose != null) {
                // Smooth correction (weighted average)
                Pose2d currentPose = getPose();
                double alpha = RobotConfig.Control.VISION_CORRECTION_ALPHA;
                Pose2d correctedPose = new Pose2d(
                        currentPose.position.x * (1 - alpha) + visionPose.position.x * alpha,
                        currentPose.position.y * (1 - alpha) + visionPose.position.y * alpha,
                        currentPose.heading.toDouble() * (1 - alpha) + visionPose.heading.toDouble() * alpha
                );
                setPose(correctedPose);
            }
        }
        */
    }

    // ===================
    // TeleOp: Direct Control
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

        // Convert field-centric input to robot-centric
        // Road Runnerではheading=90°が前向き(+Y)なので、90°オフセットを適用
        double adjustedHeading = heading - Math.PI / 2;
        double rotX = xSpeed * Math.cos(adjustedHeading) - ySpeed * Math.sin(adjustedHeading);
        double rotY = xSpeed * Math.sin(adjustedHeading) + ySpeed * Math.cos(adjustedHeading);

        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(rotY, -rotX),
                -rotation
        ));
    }

    /**
     * Drive with robot-centric control.
     * ロボットセントリック制御でドライブする。
     */
    public void driveRobotCentric(double xSpeed, double ySpeed, double rotation) {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(ySpeed, -xSpeed),
                -rotation
        ));
    }

    /**
     * Stop all motors.
     * 全モーターを停止する。
     */
    public void stop() {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    // ===================
    // Autonomous: Trajectory Actions
    // ===================

    /**
     * Get trajectory action builder starting from current pose.
     * 現在位置からの軌道アクションビルダーを取得する。
     */
    public TrajectoryActionBuilder actionBuilder() {
        return mecanumDrive.actionBuilder(getPose());
    }

    /**
     * Get trajectory action builder starting from specified pose.
     * 指定位置からの軌道アクションビルダーを取得する。
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return mecanumDrive.actionBuilder(beginPose);
    }

    // ===================
    // AprilTag Actions
    // ===================

    /**
     * Create action to correct pose using AprilTag.
     * AprilTagを使用してposeを補正するアクションを作成する。
     *
     * @param tagId Tag ID to look for
     * @return Action that corrects pose (completes immediately)
     */
    public Action correctPoseWithTag(int tagId) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (vision == null) return false;

                Pose2d tagFieldPose = vision.getTagFieldPose(tagId);
                if (tagFieldPose == null) return false;

                AprilTagDetection detection = vision.getDetection(tagId);
                if (detection == null) {
                    packet.put("apriltag_status", "Tag " + tagId + " not found");
                    return false;
                }

                Pose2d robotPose = vision.calculateRobotPose(detection, tagFieldPose);
                if (robotPose != null) {
                    setPose(robotPose);
                    packet.put("apriltag_status", "Pose corrected");
                    packet.put("corrected_pose", robotPose.toString());
                }

                return false; // Complete immediately
            }
        };
    }

    /**
     * Create action to align to AprilTag using P control.
     * P制御を使用してAprilTagに位置合わせするアクションを作成する。
     *
     * @param tagId        Tag ID to align to
     * @param targetOffset Target offset from tag (x = forward, y = left)
     * @return Action that aligns robot to tag
     */
    public Action alignToTag(int tagId, Pose2d targetOffset) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (vision == null) return false;

                AprilTagDetection detection = vision.getDetection(tagId);
                if (detection == null) {
                    packet.put("align_status", "Tag not found");
                    stop();
                    return false;
                }

                // Current offset from tag
                double xError = detection.ftcPose.y - targetOffset.position.x;  // forward
                double yError = detection.ftcPose.x - targetOffset.position.y;  // strafe
                double headingError = Math.toRadians(detection.ftcPose.yaw) - targetOffset.heading.toDouble();

                packet.put("align_x_error", xError);
                packet.put("align_y_error", yError);
                packet.put("align_heading_error", Math.toDegrees(headingError));

                // Check if aligned
                if (Math.abs(xError) < 1 && Math.abs(yError) < 1 && Math.abs(headingError) < Math.toRadians(2)) {
                    stop();
                    packet.put("align_status", "Aligned");
                    return false;
                }

                // P control
                double kP = 0.05;
                double kPHeading = 0.5;
                double maxPower = 0.3;

                double xPower = clip(-xError * kP, -maxPower, maxPower);
                double yPower = clip(-yError * kP, -maxPower, maxPower);
                double headingPower = clip(-headingError * kPHeading, -maxPower, maxPower);

                driveRobotCentric(yPower, xPower, headingPower);
                packet.put("align_status", "Aligning...");

                return true;
            }

            private double clip(double value, double min, double max) {
                return Math.max(min, Math.min(max, value));
            }
        };
    }

    /**
     * Get underlying MecanumDrive (for advanced use).
     * 内部のMecanumDriveを取得する（上級者向け）。
     */
    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }
}
