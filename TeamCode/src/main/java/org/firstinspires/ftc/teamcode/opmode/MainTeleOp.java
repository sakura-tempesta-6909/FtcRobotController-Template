package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Main TeleOp OpMode.
 * メインTeleOp OpMode。
 */
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize robot
        Robot robot = new Robot(hardwareMap);
        robot.setTelemetry(telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // ===================
            // Drive Control
            // ===================
            double xSpeed = Robot.applyDeadZone(gamepad1.left_stick_x);
            double ySpeed = Robot.applyDeadZone(-gamepad1.left_stick_y);
            double rotation = Robot.applyDeadZone(gamepad1.right_stick_x);

            robot.drive.drive(xSpeed, ySpeed, rotation);

            // Reset IMU heading with Start button
            if (gamepad1.start) {
                robot.drive.resetHeading();
            }

            // ===================
            // Subsystem Controls (Examples)
            // ===================

            // Example: Lift control (uncomment when Lift subsystem is added)
            // if (gamepad2.dpad_up && !robot.isBusy()) {
            //     robot.runAction(robot.lift.toHigh());
            // } else if (gamepad2.dpad_down && !robot.isBusy()) {
            //     robot.runAction(robot.lift.toLow());
            // } else if (!robot.isBusy()) {
            //     robot.lift.setPower(Robot.applyDeadZone(-gamepad2.left_stick_y));
            // }

            // Example: Intake control (uncomment when Intake subsystem is added)
            // if (gamepad2.a) {
            //     robot.runAction(robot.intake.open());
            // } else if (gamepad2.b) {
            //     robot.runAction(robot.intake.close());
            // }

            // ===================
            // Update Robot
            // ===================
            robot.update();

            // Update telemetry
            telemetry.update();
        }
    }
}
