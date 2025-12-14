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
     * Initialize robot with all subsystems at origin.
     * 原点で全サブシステムを初期化する。
     */
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d(0, 0, 0));
    }

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

    /**
     * Update robot state. Call this every loop.
     * ロボットの状態を更新する。毎ループ呼び出すこと。
     */
    public void update() {
        // Update localization
        drive.updateLocalization();

        // Create telemetry packet for FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();

        // Draw robot on field view
        Pose2d pose = drive.getPose();
        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke("#3F51B5");  // Blue color
        Drawing.drawRobot(canvas, pose);

        // Add pose data to packet
        packet.put("x", pose.position.x);
        packet.put("y", pose.position.y);
        packet.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

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
        Pose2d pose = drive.getPose();
        telemetry.addData("x", "%.2f", pose.position.x);
        telemetry.addData("y", "%.2f", pose.position.y);
        telemetry.addData("heading (deg)", "%.2f", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("Actions Running", runningActions.size());
        telemetry.addData("AprilTag Visible", vision.isTagVisible());
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
