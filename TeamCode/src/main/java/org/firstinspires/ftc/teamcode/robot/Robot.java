package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.action.Action;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
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

    // Telemetry (optional)
    private Telemetry telemetry;

    /**
     * Initialize robot with all subsystems.
     * 全サブシステムでロボットを初期化する。
     */
    public Robot(HardwareMap hardwareMap) {
        // Initialize subsystems
        drive = new Drive(hardwareMap);
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

        // Run actions and remove completed ones
        runningActions.removeIf(action -> !action.run());

        // Update telemetry
        if (telemetry != null) {
            updateTelemetry();
        }
    }

    /**
     * Update telemetry with robot state.
     * ロボットの状態でtelemetryを更新する。
     */
    private void updateTelemetry() {
        telemetry.addData("Pose", drive.getPose());
        telemetry.addData("Heading (deg)", Math.toDegrees(drive.getHeading()));
        telemetry.addData("Actions Running", runningActions.size());

        // Add more telemetry as needed
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
