package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.action.Action;
import org.firstinspires.ftc.teamcode.action.SequentialAction;
import org.firstinspires.ftc.teamcode.action.SleepAction;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Main Autonomous OpMode.
 * This is a placeholder that will be enhanced after Road Runner integration.
 *
 * メインAutonomous OpMode。
 * Road Runner導入後に強化されるプレースホルダー。
 */
@Autonomous(name = "Main Auto", group = "Main")
public class MainAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize robot
        Robot robot = new Robot(hardwareMap);
        robot.setTelemetry(telemetry);

        // Build autonomous sequence
        // Road Runner導入後:
        // Action auto = new SequentialAction(
        //     robot.drive.driveTo(new Pose2d(24, 0, 0)),
        //     robot.lift.toHigh(),
        //     robot.intake.open(),
        //     new SleepAction(0.3),
        //     robot.drive.driveTo(new Pose2d(48, 24, Math.PI/2))
        // );

        // Placeholder autonomous (just waits)
        Action auto = new SequentialAction(
                new SleepAction(1.0)
                // Add actions here after Road Runner integration
        );

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pose", robot.drive.getPose());
        telemetry.update();

        // Wait for start
        waitForStart();

        // Run autonomous
        robot.runAction(auto);

        while (opModeIsActive() && robot.isBusy()) {
            robot.update();
            telemetry.update();
        }

        // Stop
        robot.drive.stop();
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}
