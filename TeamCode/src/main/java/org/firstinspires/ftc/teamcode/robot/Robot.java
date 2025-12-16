package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.lib.Drawing;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Robot class that manages all subsystems.
 * Unified entry point for both TeleOp and Autonomous.
 *
 * 全サブシステムを管理するRobotクラス。
 * TeleOpとAutonomous両方の統一エントリーポイント。
 */
public class Robot {
    // Subsystems
    public final Drive drive;
    public final Vision vision;

    // Add more subsystems here as needed:
    // public final Lift lift;
    // public final Intake intake;

    // FTC Dashboard for telemetry
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Telemetry (optional)
    private Telemetry telemetry;

    // AprilTag位置の履歴（軌跡表示用）
    private Pose lastAprilTagPose = null;

    // 位置補正の重み (0.0 = 補正なし, 1.0 = 完全にAprilTagの値を使用)
    private static final double CORRECTION_ALPHA = 0.5;

    /**
     * Initialize robot with all subsystems at specified pose.
     * 指定位置で全サブシステムを初期化する。
     */
    public Robot(HardwareMap hardwareMap, Pose initialPose) {
        // Initialize subsystems
        drive = new Drive(hardwareMap, initialPose);
        vision = new Vision(hardwareMap);

        // Connect subsystems
        drive.setVision(vision);

        // Initialize more subsystems here:
        // lift = new Lift(hardwareMap);
        // intake = new Intake(hardwareMap);
    }

    /**
     * Set telemetry for debug output.
     * デバッグ出力用のtelemetryを設定する。
     */
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // ===================
    // Path Following
    // ===================

    /**
     * Check if a path is currently being followed.
     * パスが追従中かどうかを確認する。
     */
    public boolean isBusy() {
        return drive.isBusy();
    }

    /**
     * Cancel current path following.
     * 現在のパス追従をキャンセルする。
     */
    public void cancelAllActions() {
        drive.stop();
    }

    // ===================
    // AprilTag Pose Correction
    // ===================

    /**
     * Correct odometry pose using AprilTag detection.
     * AprilTag検出を使用してオドメトリ位置を補正する。
     *
     * @return true if correction was applied, false if no AprilTag visible
     */
    public boolean correctPoseWithAprilTag() {
        Pose aprilTagPose = vision.getBestPoseEstimate();
        if (aprilTagPose == null) {
            return false;
        }

        Pose currentPose = drive.getPose();

        // 重み付き平均で補正 (急激な変化を防ぐ)
        Pose correctedPose = new Pose(
                currentPose.getX() * (1 - CORRECTION_ALPHA) + aprilTagPose.getX() * CORRECTION_ALPHA,
                currentPose.getY() * (1 - CORRECTION_ALPHA) + aprilTagPose.getY() * CORRECTION_ALPHA,
                currentPose.getHeading() * (1 - CORRECTION_ALPHA) + aprilTagPose.getHeading() * CORRECTION_ALPHA
        );

        drive.setPose(correctedPose);
        return true;
    }

    /**
     * Directly set odometry pose to AprilTag pose (no smoothing).
     * オドメトリ位置を直接AprilTag位置に設定する（スムージングなし）。
     *
     * @return true if pose was set, false if no AprilTag visible
     */
    public boolean resetPoseToAprilTag() {
        Pose aprilTagPose = vision.getBestPoseEstimate();
        if (aprilTagPose == null) {
            return false;
        }
        drive.setPose(aprilTagPose);
        return true;
    }

    // ===================
    // Main Update Loop
    // ===================

    /**
     * Update robot state. Call this every loop.
     * ロボットの状態を更新する。毎ループ呼び出すこと。
     */
    public void update() {
        // Update drive (localization + path following)
        drive.update();

        // Create telemetry packet for FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // === Odometry位置（青） ===
        Pose odometryPose = drive.getPose();
        canvas.setStroke("#3F51B5");  // Blue
        canvas.setStrokeWidth(1);
        Drawing.drawRobot(canvas, odometryPose);

        // Add odometry pose data to packet (always)
        packet.put("Odometry x", odometryPose.getX());
        packet.put("Odometry y", odometryPose.getY());
        packet.put("heading (deg)", Math.toDegrees(odometryPose.getHeading()));

        // === AprilTag位置（緑）- 常に検出・表示 ===
        List<AprilTagDetection> detections = vision.getAllDetections();
        packet.put("AprilTag count", detections.size());

        if (!detections.isEmpty()) {
            AprilTagDetection det = detections.get(0);  // 最初の検出
            packet.put("Tag ID", det.id);
            packet.put("Tag range (cm)", det.ftcPose.range / RobotConfig.CM_TO_INCH);

            Pose aprilTagPose = vision.getBestPoseEstimate();
            if (aprilTagPose != null) {
                canvas.setStroke("#4CAF50");  // Green
                canvas.setStrokeWidth(1);
                Drawing.drawRobot(canvas, aprilTagPose);
                lastAprilTagPose = aprilTagPose;

                // 両者の差を計算
                double dx = aprilTagPose.getX() - odometryPose.getX();
                double dy = aprilTagPose.getY() - odometryPose.getY();
                double distCm = Math.sqrt(dx * dx + dy * dy) / RobotConfig.CM_TO_INCH;

                packet.put("AprilTag x (cm)", aprilTagPose.getX() / RobotConfig.CM_TO_INCH);
                packet.put("AprilTag y (cm)", aprilTagPose.getY() / RobotConfig.CM_TO_INCH);
                packet.put("AprilTag heading (deg)", Math.toDegrees(aprilTagPose.getHeading()));
                packet.put("Difference (cm)", distCm);
            }
        } else if (lastAprilTagPose != null) {
            // AprilTagが見えない時は最後の位置を薄く表示
            canvas.setStroke("#A5D6A7");  // Light green
            canvas.setStrokeWidth(1);
            Drawing.drawRobot(canvas, lastAprilTagPose);
        }

        // Send packet to dashboard
        dashboard.sendTelemetryPacket(packet);

        // Update local telemetry
        if (telemetry != null) {
            updateTelemetry();
        }
    }

    /**
     * Update telemetry with robot state.
     * ロボットの状態でtelemetryを更新する。
     */
    private void updateTelemetry() {
        // Odometry position (Blue on Dashboard)
        Pose odometryPose = drive.getPose();
        double odoCm_x = odometryPose.getX() / RobotConfig.CM_TO_INCH;
        double odoCm_y = odometryPose.getY() / RobotConfig.CM_TO_INCH;
        double odoHeading = Math.toDegrees(odometryPose.getHeading());

        telemetry.addLine("--- Odometry (Blue) ---");
        telemetry.addData("Position", "(%.0f, %.0f) cm, %.0f°", odoCm_x, odoCm_y, odoHeading);

        // AprilTag position (Green on Dashboard)
        List<AprilTagDetection> detections = vision.getAllDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection det = detections.get(0);
            Pose aprilTagPose = vision.getBestPoseEstimate();

            telemetry.addLine("--- AprilTag (Green) ---");
            telemetry.addData("Tag", "ID %d, %.0f cm", det.id, det.ftcPose.range / RobotConfig.CM_TO_INCH);

            if (aprilTagPose != null) {
                double tagCm_x = aprilTagPose.getX() / RobotConfig.CM_TO_INCH;
                double tagCm_y = aprilTagPose.getY() / RobotConfig.CM_TO_INCH;
                double tagHeading = Math.toDegrees(aprilTagPose.getHeading());
                telemetry.addData("Position", "(%.0f, %.0f) cm, %.0f°", tagCm_x, tagCm_y, tagHeading);

                // Difference
                double dx = aprilTagPose.getX() - odometryPose.getX();
                double dy = aprilTagPose.getY() - odometryPose.getY();
                double distCm = Math.sqrt(dx * dx + dy * dy) / RobotConfig.CM_TO_INCH;
                telemetry.addData("Diff", "%.1f cm (Y to correct)", distCm);
            }
        } else {
            telemetry.addData("AprilTag", "Not visible");
        }
    }

    // ===================
    // Utility Methods
    // ===================

    /**
     * Apply dead zone to controller input.
     * コントローラー入力にデッドゾーンを適用する。
     */
    public static double applyDeadZone(double value) {
        if (Math.abs(value) < RobotConfig.Control.DEAD_ZONE) {
            return 0.0;
        }
        return value;
    }
}
