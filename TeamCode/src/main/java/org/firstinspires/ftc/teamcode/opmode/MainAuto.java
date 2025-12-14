package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Main Autonomous OpMode using Road Runner.
 * Road Runnerを使用するメインAutonomous OpMode。
 */
@Autonomous(name = "Main Auto", group = "Main")
public class MainAuto extends OpMode {

    // Starting pose
    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);

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
        return new SequentialAction(
                // Step 1: Drive forward
                robot.drive.actionBuilder(START_POSE)
                        .lineToX(24)
                        .build(),

                // Step 2: Wait briefly
                new SleepAction(0.5),

                // Step 3: Strafe right
                robot.drive.actionBuilder(new Pose2d(24, 0, 0))
                        .strafeTo(new Vector2d(24, -24))
                        .build(),

                // Step 4: Correct pose with AprilTag (if visible)
                robot.drive.correctPoseWithTag(1),

                // Step 5: Turn and drive to scoring position
                robot.drive.actionBuilder(new Pose2d(24, -24, 0))
                        .turnTo(Math.toRadians(90))
                        .lineToY(-48)
                        .build()

                // Add more steps as needed:
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
        return new SequentialAction(
                // Drive while preparing scoring mechanism
                new ParallelAction(
                        robot.drive.actionBuilder(START_POSE)
                                .splineTo(new Vector2d(36, 24), Math.toRadians(45))
                                .build()
                        // robot.lift.toHigh()  // Uncomment when Lift is added
                ),

                // Align to AprilTag for precise scoring
                robot.drive.alignToTag(1, new Pose2d(15, 0, 0)),

                // Score
                // robot.intake.open(),
                new SleepAction(0.3),

                // Return to starting area
                new ParallelAction(
                        robot.drive.actionBuilder(robot.drive.getPose())
                                .lineToX(0)
                                .build()
                        // robot.lift.toLow()  // Uncomment when Lift is added
                )
        );
    }
}
