package org.firstinspires.ftc.teamcode.lib.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(3);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotConfig.DriveMotor.RIGHT_FRONT)
            .rightRearMotorName(RobotConfig.DriveMotor.RIGHT_REAR)
            .leftRearMotorName(RobotConfig.DriveMotor.LEFT_REAR)
            .leftFrontMotorName(RobotConfig.DriveMotor.LEFT_FRONT)
            .leftFrontMotorDirection(RobotConfig.DriveMotor.LEFT_FRONT_DIR)
            .leftRearMotorDirection(RobotConfig.DriveMotor.LEFT_REAR_DIR)
            .rightFrontMotorDirection(RobotConfig.DriveMotor.RIGHT_FRONT_DIR)
            .rightRearMotorDirection(RobotConfig.DriveMotor.RIGHT_REAR_DIR);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName(RobotConfig.DriveMotor.RIGHT_REAR)
            .strafeEncoder_HardwareMapName(RobotConfig.DriveMotor.LEFT_REAR)
            .IMU_HardwareMapName(RobotConfig.Imu.NAME)
            .IMU_Orientation(RobotConfig.Imu.ORIENTATION)
            .strafePodX(20 * 0.393701)
            .forwardPodY(0)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardEncoderDirection(Encoder.REVERSE);
//            .forwardTicksToInches(5.36426);
//            .strafeTicksToInches(40/155);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
