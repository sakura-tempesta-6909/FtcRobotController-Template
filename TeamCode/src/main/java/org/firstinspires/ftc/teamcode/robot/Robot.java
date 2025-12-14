package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.lib.Drawing;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * Robot class that manages all subsystems and actions.
 * Unified entry point for both TeleOp and Autonomous.
 *
 * 全サブシステムとアクションを管理するRobotクラス。
 * TeleOpとAutonomous両方の統一エントリーポイント。
 */
public class Robot {
    // Subsystems
    public final Drive drive;
    public final Vision vision;

    // Add more subsystems here as needed:
    // public final Lift lift;
    // public final Intake intake;

    // Action management
    private final List<Action> runningActions = new ArrayList<>();

    // FTC Dashboard for telemetry
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Telemetry (optional)
    private Telemetry telemetry;

    /**
     * Initialize robot with all subsystems at specified pose.
     * 指定位置で全サブシステムを初期化する。
     */
    public Robot(HardwareMap hardwareMap, Pose2d initialPose) {
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
    // Action Management
    // ===================

    /**
     * Add an action to the running queue.
     * アクションを実行キューに追加する。
     */
    public void runAction(Action action) {
        runningActions.add(action);
    }

    /**
     * Check if any actions are currently running.
     * アクションが実行中かどうかを確認する。
     */
    public boolean isBusy() {
        return !runningActions.isEmpty();
    }

    /**
     * Cancel all running actions.
     * 全ての実行中アクションをキャンセルする。
     */
    public void cancelAllActions() {
        runningActions.clear();
        drive.stop();
    }

    // ===================
    // Main Update Loop
    // ===================

    // AprilTag位置の履歴（軌跡表示用）
    private Pose2d lastAprilTagPose = null;

    // AprilTag処理を有効にするかどうか（表示のみ、位置補正なし）
    // Whether to enable AprilTag processing (display only, no pose correction)
    private boolean aprilTagEnabled = true;

    /**
     * Enable or disable AprilTag processing.
     * AprilTag処理を有効/無効にする。
     */
    public void setAprilTagEnabled(boolean enabled) {
        this.aprilTagEnabled = enabled;
    }

    /**
     * Update robot state. Call this every loop.
     * ロボットの状態を更新する。毎ループ呼び出すこと。
     */
    public void update() {
        // Update localization (DeadWheel only - AprilTag does NOT affect this)
        // 自己位置推定を更新（デッドホイールのみ - AprilTagは影響しない）
        drive.updateLocalization();

        // Create telemetry packet for FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // === DeadWheel位置（青） ===
        Pose2d odometryPose = drive.getPose();
        canvas.setStroke("#3F51B5");  // Blue
        canvas.setStrokeWidth(2);
        Drawing.drawRobot(canvas, odometryPose);

        // Add odometry pose data to packet (always)
        packet.put("Odometry x", odometryPose.position.x);
        packet.put("Odometry y", odometryPose.position.y);
        packet.put("heading (deg)", Math.toDegrees(odometryPose.heading.toDouble()));

        // === AprilTag位置（緑）- 表示のみ ===
        if (aprilTagEnabled) {
            // デバッグ: 検出された生データを表示
            List<AprilTagDetection> detections = vision.getAllDetections();
            packet.put("AprilTag count", detections.size());

            if (!detections.isEmpty()) {
                AprilTagDetection det = detections.get(0);  // 最初の検出
                packet.put("Tag ID", det.id);
                packet.put("Tag range (in)", det.ftcPose.range);
                packet.put("Tag x (in)", det.ftcPose.x);  // 横方向
                packet.put("Tag y (in)", det.ftcPose.y);  // 前方向
                packet.put("Tag yaw (deg)", det.ftcPose.yaw);
                packet.put("Tag bearing (deg)", det.ftcPose.bearing);

                Pose2d aprilTagPose = vision.getBestPoseEstimate();
                if (aprilTagPose != null) {
                    canvas.setStroke("#4CAF50");  // Green
                    canvas.setStrokeWidth(2);
                    Drawing.drawRobot(canvas, aprilTagPose);
                    lastAprilTagPose = aprilTagPose;

                    // 両者の差を計算
                    double dx = aprilTagPose.position.x - odometryPose.position.x;
                    double dy = aprilTagPose.position.y - odometryPose.position.y;
                    double dist = Math.sqrt(dx * dx + dy * dy);

                    packet.put("Calc Robot x", aprilTagPose.position.x);
                    packet.put("Calc Robot y", aprilTagPose.position.y);
                    packet.put("Calc heading (deg)", Math.toDegrees(aprilTagPose.heading.toDouble()));
                    packet.put("Difference (in)", dist);
                }
            } else if (lastAprilTagPose != null) {
                // AprilTagが見えない時は最後の位置を薄く表示
                canvas.setStroke("#A5D6A7");  // Light green
                canvas.setStrokeWidth(1);
                Drawing.drawRobot(canvas, lastAprilTagPose);
                packet.put("AprilTag", "Not visible (showing last)");
            } else {
                packet.put("AprilTag", "Not visible");
            }
        } else {
            packet.put("AprilTag", "Disabled");
        }

        // Run actions and remove completed ones
        runningActions.removeIf(action -> !action.run(packet));

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
        // DeadWheel (Odometry)
        Pose2d odometryPose = drive.getPose();
        telemetry.addLine("=== DeadWheel (Blue) ===");
        telemetry.addData("  x", "%.2f in", odometryPose.position.x);
        telemetry.addData("  y", "%.2f in", odometryPose.position.y);
        telemetry.addData("  heading", "%.1f°", Math.toDegrees(odometryPose.heading.toDouble()));

        // AprilTag
        if (aprilTagEnabled) {
            List<AprilTagDetection> detections = vision.getAllDetections();
            telemetry.addLine("=== AprilTag Raw Data ===");
            telemetry.addData("  Count", detections.size());

            if (!detections.isEmpty()) {
                AprilTagDetection det = detections.get(0);
                telemetry.addData("  Tag ID", det.id);
                telemetry.addData("  range", "%.1f in", det.ftcPose.range);
                telemetry.addData("  x (right+)", "%.1f in", det.ftcPose.x);
                telemetry.addData("  y (forward+)", "%.1f in", det.ftcPose.y);
                telemetry.addData("  yaw", "%.1f°", det.ftcPose.yaw);
                telemetry.addData("  bearing", "%.1f°", det.ftcPose.bearing);

                Pose2d aprilTagPose = vision.getBestPoseEstimate();
                if (aprilTagPose != null) {
                    telemetry.addLine("=== Calculated Robot Pose ===");
                    telemetry.addData("  x", "%.2f in", aprilTagPose.position.x);
                    telemetry.addData("  y", "%.2f in", aprilTagPose.position.y);
                    telemetry.addData("  heading", "%.1f°", Math.toDegrees(aprilTagPose.heading.toDouble()));

                    // 差分
                    double dx = aprilTagPose.position.x - odometryPose.position.x;
                    double dy = aprilTagPose.position.y - odometryPose.position.y;
                    double dist = Math.sqrt(dx * dx + dy * dy);
                    telemetry.addLine("=== Difference ===");
                    telemetry.addData("  Δx", "%.2f in", dx);
                    telemetry.addData("  Δy", "%.2f in", dy);
                    telemetry.addData("  Distance", "%.2f in", dist);
                }
            } else {
                telemetry.addData("  Status", "Not visible");
            }
        } else {
            telemetry.addLine("=== AprilTag ===");
            telemetry.addData("  Status", "Disabled (Y to enable)");
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
