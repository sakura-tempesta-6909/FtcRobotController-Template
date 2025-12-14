package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Main TeleOp OpMode.
 * メインTeleOp OpMode。
 */
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    // ループ時間計測用
    private long lastLoopTime = 0;
    private double loopTimeMs = 0;
    private double avgLoopTimeMs = 0;
    private static final double AVG_ALPHA = 0.1;  // 移動平均の係数

    // AprilTagトグル用
    private boolean aprilTagEnabled = false;
    private boolean lastY = false;
    private boolean lastX = false;

    // ===================
    // 初期位置の設定（MeepMeepと同じ座標系）
    // ===================
    // cm単位で入力、自動でインチに変換
    // x: 左右（正 = 右）
    // y: 前後（正 = 奥）
    // heading: 向き（度、0 = 右、90 = 奥向き）
    private static final double START_X_CM = 10.0;
//    private static final double START_Y_CM = -163.0;
    private static final double START_Y_CM = -163.0 -61 ;
    private static final double START_HEADING_DEG = 90.0;

    private static final Pose2d START_POSE = new Pose2d(
            START_X_CM * RobotConfig.CM_TO_INCH,
            START_Y_CM * RobotConfig.CM_TO_INCH,
            Math.toRadians(START_HEADING_DEG)
    );

    private Robot robot;

    /**
     * Called once when INIT is pressed.
     * INITが押されたときに1回呼ばれる。
     */
    @Override
    public void init() {
        robot = new Robot(hardwareMap, START_POSE);
        robot.setTelemetry(telemetry);

        // AprilTagはデフォルトで無効（Yボタンでトグル）
        // AprilTag disabled by default (toggle with Y button)
        robot.setAprilTagEnabled(false);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", START_POSE.toString());
        telemetry.addData("AprilTag", "Disabled (press Y to toggle)");
    }

    /**
     * Called repeatedly between INIT and START.
     * INITからSTARTの間、繰り返し呼ばれる。
     */
    @Override
    public void init_loop() {
        // AprilTagは無効のままupdateを呼ぶ
        robot.update();

        telemetry.addData("Pose", robot.drive.getPose());
        telemetry.addData("AprilTag", "Disabled (press Y after START to enable)");
        telemetry.update();
    }

    /**
     * Called once when START is pressed.
     * STARTが押されたときに1回呼ばれる。
     */
    @Override
    public void start() {
        telemetry.addData("Status", "Running");
    }

    /**
     * Called repeatedly while OpMode is active.
     * OpModeがアクティブな間、繰り返し呼ばれる。
     */
    @Override
    public void loop() {
        // ループ時間計測開始
        long currentTime = System.nanoTime();
        if (lastLoopTime != 0) {
            loopTimeMs = (currentTime - lastLoopTime) / 1_000_000.0;
            avgLoopTimeMs = avgLoopTimeMs * (1 - AVG_ALPHA) + loopTimeMs * AVG_ALPHA;
        }
        lastLoopTime = currentTime;

        // ===================
        // Drive Control（手動操作）
        // ===================
        double xSpeed = Robot.applyDeadZone(gamepad1.left_stick_x);
        double ySpeed = Robot.applyDeadZone(-gamepad1.left_stick_y);
        double rotation = Robot.applyDeadZone(gamepad1.right_stick_x);

        // Actionが実行中でなければ手動操作（フィールドセントリック）
        if (!robot.isBusy()) {
            robot.drive.drive(xSpeed, ySpeed, rotation);
        }

        // ===================
        // Autonomous Actions（ボタンで明示的に実行）
        // ===================

        // 例: 射出位置に自動移動（Aボタン）
        // if (gamepad1.a && !robot.isBusy()) {
        //     robot.runAction(robot.drive.actionBuilder()
        //         .splineTo(new Vector2d(48, 24), Math.toRadians(90))
        //         .build());
        // }

        // キャンセル（Bボタン）
        if (gamepad1.b) {
            robot.cancelAllActions();
        }

        // AprilTagトグル（Yボタン）
        // Toggle AprilTag (Y button)
        if (gamepad1.y && !lastY) {
            aprilTagEnabled = !aprilTagEnabled;
            robot.setAprilTagEnabled(aprilTagEnabled);
        }
        lastY = gamepad1.y;

        // 露出設定を再適用（Xボタン）
        // Dashboardで値を変更後にXを押すと反映
        if (gamepad1.x && !lastX) {
            robot.vision.applyExposureSettings();
        }
        lastX = gamepad1.x;

        // ===================
        // Subsystem Controls
        // ===================

        // 例: Lift control
        // robot.lift.setPower(Robot.applyDeadZone(-gamepad2.left_stick_y));

        // ===================
        // Update Robot
        // （この中でAprilTagによる位置補正が自動で行われる）
        // ===================
        robot.update();

        // ループ時間をテレメトリに表示
        telemetry.addData("Loop Time", "%.1f ms (avg: %.1f ms)", loopTimeMs, avgLoopTimeMs);
        telemetry.addData("Loop Rate", "%.1f Hz", 1000.0 / avgLoopTimeMs);
        telemetry.addData("AprilTag", aprilTagEnabled ? "Enabled (Y to disable)" : "Disabled (Y to enable)");
        telemetry.addData("Exposure", robot.vision.getExposureInfo() + " (X to apply)");
        telemetry.update();
    }

    /**
     * Called once when STOP is pressed.
     * STOPが押されたときに1回呼ばれる。
     */
    @Override
    public void stop() {
        robot.drive.stop();
    }
}
