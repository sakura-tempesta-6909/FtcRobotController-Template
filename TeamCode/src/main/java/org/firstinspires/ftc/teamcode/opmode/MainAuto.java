package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Main Autonomous OpMode using Pedro Pathing.
 * Pedro Pathingを使用するメインAutonomous OpMode。
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

    private static final Pose START_POSE = new Pose(
            START_X_CM * RobotConfig.CM_TO_INCH,
            START_Y_CM * RobotConfig.CM_TO_INCH,
            Math.toRadians(START_HEADING_DEG)
    );

    private Robot robot;
    private PathChain autoPath;

    // State machine for autonomous
    private enum AutoState {
        IDLE,
        FOLLOWING_PATH,
        CORRECTING_POSE,
        COMPLETE
    }
    private AutoState currentState = AutoState.IDLE;

    /**
     * Called once when INIT is pressed.
     * INITが押されたときに1回呼ばれる。
     */
    @Override
    public void init() {
        robot = new Robot(hardwareMap, START_POSE);
        robot.setTelemetry(telemetry);

        // Build autonomous path
        autoPath = buildAutonomousPath();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", "(%.0f, %.0f) cm, %.0f°",
                START_X_CM, START_Y_CM, START_HEADING_DEG);
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
        // Start following the path
        robot.drive.followPath(autoPath, true);  // holdEnd = true
        currentState = AutoState.FOLLOWING_PATH;

        telemetry.addData("Status", "Running");
    }

    /**
     * Called repeatedly while OpMode is active.
     * OpModeがアクティブな間、繰り返し呼ばれる。
     */
    @Override
    public void loop() {
        robot.update();

        // State machine
        switch (currentState) {
            case FOLLOWING_PATH:
                if (!robot.isBusy()) {
                    // Path complete, try to correct with AprilTag
                    currentState = AutoState.CORRECTING_POSE;
                }
                break;

            case CORRECTING_POSE:
                // Try AprilTag correction
                robot.correctPoseWithAprilTag();
                currentState = AutoState.COMPLETE;
                break;

            case COMPLETE:
                telemetry.addData("Status", "Complete");
                break;

            default:
                break;
        }

        telemetry.addData("State", currentState.toString());
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
     * Build autonomous path.
     * Autonomousパスを構築する。
     */
    private PathChain buildAutonomousPath() {
        // Example: Simple forward movement and turn
        // 例: シンプルな前進と回転

        // Target positions (in cm, converted to inches)
        Pose waypoint1 = new Pose(
                120 * RobotConfig.CM_TO_INCH,
                33 * RobotConfig.CM_TO_INCH,
                Math.toRadians(145)
        );

        Pose waypoint2 = new Pose(
                100 * RobotConfig.CM_TO_INCH,
                50 * RobotConfig.CM_TO_INCH,
                Math.toRadians(90)
        );

        return robot.drive.pathBuilder()
                // Move to waypoint1 with linear heading interpolation
                .addPath(new BezierLine(START_POSE, waypoint1))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), waypoint1.getHeading())

                // Line to waypoint2 (use BezierLine for simple path)
                .addPath(new BezierLine(waypoint1, waypoint2))
                .setLinearHeadingInterpolation(waypoint1.getHeading(), waypoint2.getHeading())

                .build();
    }

    /**
     * Example: Build a more complex autonomous with multiple phases.
     * 例: 複数フェーズを持つより複雑なAutonomous。
     */
    @SuppressWarnings("unused")
    private PathChain buildComplexAutonomousPath() {
        // This is just an example of how to build paths
        // 実際の座標はフィールドに合わせて調整

        Pose scoringPose = new Pose(
                36 * RobotConfig.CM_TO_INCH,
                24 * RobotConfig.CM_TO_INCH,
                Math.toRadians(45)
        );

        Pose parkPose = new Pose(
                60 * RobotConfig.CM_TO_INCH,
                60 * RobotConfig.CM_TO_INCH,
                Math.toRadians(0)
        );

        // Control point for BezierCurve (heading is ignored for control points)
        Pose controlPoint = new Pose(100 * RobotConfig.CM_TO_INCH, 40 * RobotConfig.CM_TO_INCH, 0);

        return robot.drive.pathBuilder()
                // Phase 1: Go to scoring position with curve
                .addPath(new BezierCurve(START_POSE, controlPoint, scoringPose))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), scoringPose.getHeading())

                // Phase 2: Go to park
                .addPath(new BezierLine(scoringPose, parkPose))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), parkPose.getHeading())

                .build();
    }
}
