package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.state.State;
import org.firstinspires.ftc.teamcode.subClass.Const;
public class Outtake implements Component {
    public final Servo outtakeCollector;
    public final Servo outtakeLiftLeft;
    public final Servo outtakeLiftRight;
    public final DcMotor outtakeSliderLeft;
    public final DcMotor outtakeSliderRight;
    public final Servo outtakeRotation;
    public Outtake(HardwareMap hardwareMap) {
        outtakeCollector = hardwareMap.get(Servo.class, Const.outtake.Name.Collector);
        outtakeCollector.setDirection(Servo.Direction.REVERSE);
        outtakeCollector.setPosition(Const.outtake.Position.collectorInit);

        outtakeLiftLeft = hardwareMap.get(Servo.class, Const.outtake.Name.liftLeft);
        outtakeLiftLeft.setDirection(Servo.Direction.REVERSE);
        outtakeLiftLeft.setPosition(Const.outtake.Position.liftInit);

        outtakeLiftRight = hardwareMap.get(Servo.class, Const.outtake.Name.liftRight);
        outtakeLiftRight.setDirection(Servo.Direction.FORWARD);
        outtakeLiftRight.setPosition(Const.outtake.Position.liftInit);

        outtakeRotation = hardwareMap.get(Servo.class, Const.outtake.Name.Rotation);
        outtakeRotation.setDirection(Servo.Direction.FORWARD);
        outtakeRotation.setPosition(Const.outtake.Position.rotationInit);

        outtakeSliderLeft = hardwareMap.get(DcMotor.class, Const.outtake.Name.sliderLeft);
        outtakeSliderRight = hardwareMap.get(DcMotor.class, Const.outtake.Name.sliderRight);
        outtakeSliderLeft.setDirection(Const.outtake.Direction.sliderLeft);
        outtakeSliderRight.setDirection(Const.outtake.Direction.sliderRight);
        outtakeSliderLeft.setMode(Const.outtake.Mode.sliderInit);
        outtakeSliderRight.setMode(Const.outtake.Mode.sliderInit);
        outtakeSliderLeft.setTargetPosition(Const.outtake.Position.sliderInit);
        outtakeSliderRight.setTargetPosition(Const.outtake.Position.sliderInit);
        outtakeSliderLeft.setPower(Const.outtake.Power.sliderInit);
        outtakeSliderRight.setPower(Const.outtake.Power.sliderInit);
        outtakeSliderLeft.setMode(Const.outtake.Mode.sliderMoving);
        outtakeSliderRight.setMode(Const.outtake.Mode.sliderMoving);

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void readSensors(State state) {
        state.slideState.portHandRotationServoNumber = outtakeRotation.getPortNumber();
        state.slideState.currentHandRotationServoPosition = outtakeRotation.getController().getServoPosition(outtakeRotation.getPortNumber());
//                outtakeRotation.getPosition();
        state.driveState.sliderLeftPosition = outtakeSliderLeft.getCurrentPosition();
        state.driveState.sliderRightPosition = outtakeSliderRight.getCurrentPosition();

    }

    @Override
    public void applyState(State state) {
         if (state.driveState.sliderIsUp) {
             outtakeSliderLeft.setTargetPosition(Const.outtake.Position.sliderUp);
             outtakeSliderRight.setTargetPosition(Const.outtake.Position.sliderUp);
             outtakeSliderLeft.setPower(Const.outtake.Power.sliderMoving);
             outtakeSliderRight.setPower(Const.outtake.Power.sliderMoving);
         }
//         }else{
//                 SLideLeft.setTargetPosition(0);
//                 SLideRight.setTargetPosition(0);
//                 SLideLeft.setPower(0.5);
//                 SLideRight.setPower(0.5);
//         }

        //        if (state.driveState.climb){
//            climbLeft.setTargetPosition(30);
//            climbRight.setTargetPosition(30);
//            climbLeft.setPower(0.5);
//            climbRight.setPower(0.5);
//        }else{
//            climbLeft.setTargetPosition(0);
//            climbRight.setTargetPosition(0);
//            climbLeft.setPower(0.5);
//            climbRight.setPower(0.5);
//        }

        if (state.driveState.outtakeCollectorIsUP){
            outtakeLiftLeft.setPosition(Const.outtake.Position.liftUp);
            outtakeLiftRight.setPosition(Const.outtake.Position.liftUp);
        }else{
            outtakeLiftLeft.setPosition(Const.outtake.Position.liftInit);
            outtakeLiftRight.setPosition(Const.outtake.Position.liftInit);
        }

        if (state.driveState.outtakeCollectorIsOpen){
            outtakeCollector.setPosition(Const.outtake.Position.collectorOpen);
        }else{
            outtakeCollector.setPosition(Const.outtake.Position.collectorInit);
        }

        if (state.driveState.outtakeCollectorIsRolling){
            outtakeRotation.setPosition(Const.outtake.Position.rotationUp);
        }else{
            outtakeRotation.setPosition(Const.outtake.Position.rotationInit);
        }
    }
}
