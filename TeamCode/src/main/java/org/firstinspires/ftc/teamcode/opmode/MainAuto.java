package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Main Autonomous OpMode using Road Runner.
 * Road Runnerを使用するメインAutonomous OpMode。
 *
 * 座標系 (FTC DECODE):
 * - X軸: Goal側 (X-) ↔ Audience Wall側 (X+)
 * - Y軸: Red Wall (Y-) ↔ Blue Wall (Y+)
 * - Heading: 0° = X+方向, 90° = Y+方向
 */
@Autonomous(name = "Main Auto", group = "Main")
public class MainAuto extends OpMode {

    // Starting pose (cm単位で設定、インチに変換)
    // Red Alliance側スタート: X=152cm, Y=33cm, Heading=145° (Blue方向)
    private static final double START_X_CM = 152.0;
    private static final double START_Y_CM = 33.0;
    private static final double START_HEADING_DEG = 145.0;

    private static final Pose2d START_POSE = new Pose2d(
            START_X_CM * RobotConfig.CM_TO_INCH,
            START_Y_CM * RobotConfig.CM_TO_INCH,
            Math.toRadians(START_HEADING_DEG)
    );

    private Robot robot;
    private boolean autoStarted = false;

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
     */
    @Override
    public void init_loop() {
        robot.update();
        telemetry.update();
    }

    /**
     * Called once when START is pressed.
     * STARTが押されたときに1回呼ばれる。
     */
    @Override
    public void start() {
        // Build and start autonomous sequence
        Action auto = buildAutonomous();
        robot.runAction(auto);
        autoStarted = true;

        telemetry.addData("Status", "Running");
    }

    /**
     * Called repeatedly while OpMode is active.
     * OpModeがアクティブな間、繰り返し呼ばれる。
     */
    @Override
    public void loop() {
        robot.update();

        if (autoStarted && !robot.isBusy()) {
            telemetry.addData("Status", "Complete");
        }

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

    /**
     * Build autonomous action sequence.
     * Autonomousアクションシーケンスを構築する。
     */
    private Action buildAutonomous() {
        // MeepMeep座標系: START_POSE = (0, -65, -90°)
        // ロボットはフィールド下部中央、フィールド中心方向を向いている
        return new SequentialAction(
                // Step 1: フィールド中心に向かって前進
                robot.drive.actionBuilder(START_POSE)
                        .lineToY(-40)
                        .build(),

                // Step 2: 少し待機
                new SleepAction(0.5),

                // Step 3: 右にストレイフ
                robot.drive.actionBuilder(new Pose2d(0, -40, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(24, -40))
                        .build(),

                // Step 4: AprilTagで位置補正（見える場合）
                robot.drive.correctPoseWithTag(1),

                // Step 5: 回転してスコアリング位置へ
                robot.drive.actionBuilder(new Pose2d(24, -40, Math.toRadians(-90)))
                        .turnTo(Math.toRadians(0))
                        .lineToX(48)
                        .build()

                // 必要に応じてステップを追加:
                // robot.lift.toHigh(),
                // robot.intake.open(),
                // new SleepAction(0.3),
        );
    }

    /**
     * Example: Build a more complex autonomous with parallel actions.
     * 例: 並列アクションを使ったより複雑なAutonomous。
     */
    @SuppressWarnings("unused")
    private Action buildComplexAutonomous() {
        // MeepMeep座標系での複雑なAutonomous例
        return new SequentialAction(
                // 移動しながらスコアリング機構を準備
                new ParallelAction(
                        robot.drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(36, -24), Math.toRadians(45))
                                .build()
                        // robot.lift.toHigh()  // Liftを追加したらコメント解除
                ),

                // AprilTagに合わせて精密なスコアリング
                robot.drive.alignToTag(1, new Pose2d(15, 0, 0)),

                // スコア
                // robot.intake.open(),
                new SleepAction(0.3),

                // スタートエリアに戻る
                new ParallelAction(
                        robot.drive.actionBuilder(robot.drive.getPose())
                                .lineToY(-60)
                                .build()
                        // robot.lift.toLow()  // Liftを追加したらコメント解除
                )
        );
    }
}
