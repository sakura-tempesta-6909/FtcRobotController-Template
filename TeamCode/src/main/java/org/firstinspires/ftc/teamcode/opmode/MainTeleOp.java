package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Main TeleOp OpMode.
 * メインTeleOp OpMode。
 *
 * ボタン:
 * - Y: AprilTagで位置を補正
 * - X: 露出設定を適用
 * - B: 全アクションをキャンセル
 */
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    // ループ時間計測用
    private long lastLoopTime = 0;
    private double loopTimeMs = 0;
    private double avgLoopTimeMs = 0;
    private static final double AVG_ALPHA = 0.1;

    // ボタン状態
    private boolean lastY = false;
    private boolean lastX = false;

    // 位置補正フィードバック
    private String correctionStatus = "";
    private long correctionStatusTime = 0;

    // ===================
    // 初期位置の設定
    // ===================
    // FTC DECODE座標系: X+ = Audience Wall, X- = Goal, Y+ = Blue Wall, Y- = Red Wall
    private static final double START_X_CM = 152.0;
    private static final double START_Y_CM = 33.0;
    private static final double START_HEADING_DEG = 145.0;

    private static final Pose START_POSE = new Pose(
            START_X_CM * RobotConfig.CM_TO_INCH,
            START_Y_CM * RobotConfig.CM_TO_INCH,
            Math.toRadians(START_HEADING_DEG)
    );

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, START_POSE);
        robot.setTelemetry(telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start", "(%.0f, %.0f) cm, %.0f°", START_X_CM, START_Y_CM, START_HEADING_DEG);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        robot.update();
        telemetry.update();
    }

    @Override
    public void start() {
        // Start TeleOp drive mode
        robot.drive.startTeleOp();
        telemetry.addData("Status", "Running");
    }

    @Override
    public void loop() {
        // ループ時間計測
        long currentTime = System.nanoTime();
        if (lastLoopTime != 0) {
            loopTimeMs = (currentTime - lastLoopTime) / 1_000_000.0;
            avgLoopTimeMs = avgLoopTimeMs * (1 - AVG_ALPHA) + loopTimeMs * AVG_ALPHA;
        }
        lastLoopTime = currentTime;

        // ===================
        // Drive Control
        // ===================
        double forward = Robot.applyDeadZone(-gamepad1.left_stick_y);
        double strafe = Robot.applyDeadZone(-gamepad1.left_stick_x);
        double turn = Robot.applyDeadZone(-gamepad1.right_stick_x);

        if (!robot.isBusy()) {
            robot.drive.drive(forward, strafe, turn);
        }

        // ===================
        // Button Controls
        // ===================

        // Y: AprilTagで位置を補正
        if (gamepad1.y && !lastY) {
            boolean success = robot.correctPoseWithAprilTag();
            if (success) {
                correctionStatus = "Corrected!";
            } else {
                correctionStatus = "No tag visible";
            }
            correctionStatusTime = System.currentTimeMillis();
        }
        lastY = gamepad1.y;

        // X: 露出設定を再適用
        if (gamepad1.x && !lastX) {
            robot.vision.applyExposureSettings();
        }
        lastX = gamepad1.x;

        // B: 全アクションをキャンセル
        if (gamepad1.b) {
            robot.cancelAllActions();
        }

        // ===================
        // Update Robot
        // ===================
        robot.update();

        // ===================
        // Telemetry
        // ===================
        telemetry.addData("Loop", "%.0f Hz", 1000.0 / avgLoopTimeMs);

        // 補正ステータス表示 (2秒間)
        if (System.currentTimeMillis() - correctionStatusTime < 2000 && !correctionStatus.isEmpty()) {
            telemetry.addData("Correction", correctionStatus);
        }

        telemetry.addLine("--- Controls ---");
        telemetry.addData("Y", "Correct pose with AprilTag");
        telemetry.addData("X", "Apply exposure");
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.drive.stop();
    }
}
