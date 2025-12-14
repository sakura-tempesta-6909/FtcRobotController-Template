package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Main TeleOp OpMode.
 * メインTeleOp OpMode。
 */
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    // ===================
    // 初期位置の設定
    // ===================
    // x: 前後（正 = 前）
    // y: 左右（正 = 左）
    // heading: 向き（ラジアン、0 = 前向き）
    private static final Pose2d START_POSE = new Pose2d(0, 0, Math.toRadians(0));

    private Robot robot;

    /**
     * Called once when INIT is pressed.
     * INITが押されたときに1回呼ばれる。
     */
    @Override
    public void init() {
        robot = new Robot(hardwareMap, START_POSE);
        robot.setTelemetry(telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", START_POSE.toString());
    }

    /**
     * Called repeatedly between INIT and START.
     * INITからSTARTの間、繰り返し呼ばれる。
     * AprilTagで初期位置を自動検出する。
     */
    @Override
    public void init_loop() {
        robot.update();

        // AprilTagで初期位置を検出
        if (robot.vision.isTagVisible()) {
            telemetry.addData("AprilTag", "Detected - Pose calibrated");
        } else {
            telemetry.addData("AprilTag", "Not visible - Point camera at tag");
        }

        telemetry.addData("Pose", robot.drive.getPose());
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
        // ===================
        // Drive Control（手動操作）
        // ===================
        double xSpeed = Robot.applyDeadZone(gamepad1.left_stick_x);
        double ySpeed = Robot.applyDeadZone(-gamepad1.left_stick_y);
        double rotation = Robot.applyDeadZone(gamepad1.right_stick_x);

        // Actionが実行中でなければ手動操作
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
