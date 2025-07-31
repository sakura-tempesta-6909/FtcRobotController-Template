package org.firstinspires.ftc.teamcode.subClass;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Const {
    public static class Drive {
        public static class Name {
            public static final String leftFront = "leftFront";
            public static final String rightFront = "rightFront";
            public static final String leftRear = "leftRear";
            public static final String rightRear = "rightRear";
            public static final String imu = "imu";
        }

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        public static class Direction {
            public static final DcMotor.Direction leftFront = DcMotor.Direction.REVERSE;
            public static final DcMotor.Direction rightFront = DcMotor.Direction.REVERSE;
            public static final DcMotor.Direction leftRear = DcMotor.Direction.REVERSE;
            public static final DcMotor.Direction rightRear = DcMotor.Direction.FORWARD;
        }

        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot HUB_ORIENTATION =
                new RevHubOrientationOnRobot(
                        LOGO_FACING_DIRECTION,
                        USB_FACING_DIRECTION
                );
        public static class AutonomousDrive {

        }
    }

    public static class Other{
        public static final Double CONTROLLER_DEAD_ZONE = 0.1;
    }
}
