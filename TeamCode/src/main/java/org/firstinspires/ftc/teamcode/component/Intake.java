package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.state.State;
import org.firstinspires.ftc.teamcode.subClass.Const;

public class Intake implements Component {
    private final CRServo intakeCollectorLeft;
    private final CRServo intakeCollectorRight;
    public final Servo intakeSliderLeft;
    public final Servo intakeSliderRight;
    public final Servo intakeHorizontalRotation;
    public final Servo intakeVerticalRotation;
    public final Servo intakeLiftLeft;
    public final Servo intakeLiftRight;
    public Intake(HardwareMap hardwareMap) {

        intakeCollectorLeft = hardwareMap.get(CRServo.class, Const.intake.Name.collectorLeft);
        intakeCollectorRight = hardwareMap.get(CRServo.class, Const.intake.Name.collectorRight);
        intakeCollectorRight.setPower(Const.intake.Power.collectorPowerInit);
        intakeCollectorLeft.setPower(Const.intake.Power.collectorPowerInit);
        intakeCollectorLeft.setDirection(Const.intake.Direction.collectorLeftInit);
        intakeCollectorRight.setDirection(Const.intake.Direction.collectorRightInit);

        intakeSliderLeft = hardwareMap.get(Servo.class, Const.intake.Name.sliderLeft);
        intakeSliderRight = hardwareMap.get(Servo.class, Const.intake.Name.sliderRight);
        intakeSliderLeft.setDirection(Const.intake.Direction.sliderLeft);
        intakeSliderRight.setDirection(Const.intake.Direction.sliderRight);
        intakeSliderLeft.setPosition(Const.intake.Position.sliderInit);
        intakeSliderRight.setPosition(Const.intake.Position.sliderInit);

        intakeHorizontalRotation = hardwareMap.get(Servo.class, Const.intake.Name.horizontalRotation);
        intakeHorizontalRotation.setPosition(Const.intake.Position.horizontalRotationInit);
        intakeHorizontalRotation.setDirection(Servo.Direction.REVERSE);

        intakeVerticalRotation =  hardwareMap.get(Servo.class, Const.intake.Name.verticalRotation);
        intakeVerticalRotation.setPosition(Const.intake.Position.verticalRotationInit);
        intakeVerticalRotation.setDirection(Servo.Direction.FORWARD);

        intakeLiftLeft = hardwareMap.get(Servo.class, Const.intake.Name.liftLeft);
        intakeLiftLeft.setDirection(Const.intake.Direction.liftLeft);
        intakeLiftLeft.setPosition(Const.intake.Position.liftInit);

        intakeLiftRight = hardwareMap.get(Servo.class, Const.intake.Name.liftRight);
        intakeLiftRight.setDirection(Const.intake.Direction.liftRight);
        intakeLiftRight.setPosition(Const.intake.Position.liftInit);
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

    }

    @Override
    public void applyState(State state) {

        intakeHorizontalRotation.setDirection(Servo.Direction.FORWARD);

        if (state.driveState.charge){
            intakeCollectorRight.setPower(Const.intake.Power.ArmPowerCharge);
            intakeCollectorLeft.setPower(Const.intake.Power.ArmPowerCharge);
        }else if(state.driveState.discharge) {
            intakeCollectorRight.setPower(Const.intake.Power.ArmPowerDischarge);
            intakeCollectorLeft.setPower(Const.intake.Power.ArmPowerDischarge);
        }else{
            intakeCollectorRight.setPower(Const.intake.Power.collectorPowerInit);
            intakeCollectorLeft.setPower(Const.intake.Power.collectorPowerInit);
        }

        if (state.driveState.intakeRotation){
            intakeHorizontalRotation.setPosition(Const.intake.Position.horizontalRotationMoving);
        }else{
            intakeHorizontalRotation.setPosition(Const.intake.Position.horizontalRotationInit);
        }

        if (state.controllerState.robotCharge) {
            intakeVerticalRotation.setPosition(Const.intake.Position.verticalRotationSide);
            intakeSliderLeft.setPosition(Const.intake.Position.sliderHead);
            intakeSliderRight.setPosition(Const.intake.Position.sliderHead);
            if(!state.driveState.charge){
                intakeLiftLeft.setPosition(Const.intake.Position.liftLimited);
                intakeLiftRight.setPosition(Const.intake.Position.liftLimited);
            }else{
                intakeLiftLeft.setPosition(Const.intake.Position.liftLowest);
                intakeLiftRight.setPosition(Const.intake.Position.liftLowest);
            }
        } else {
            intakeVerticalRotation.setPosition(Const.intake.Position.verticalRotationInit);
            intakeSliderLeft.setPosition(Const.intake.Position.sliderInit);
            intakeSliderRight.setPosition(Const.intake.Position.sliderInit);
            intakeLiftLeft.setPosition(Const.intake.Position.liftInit);
            intakeLiftRight.setPosition(Const.intake.Position.liftInit);
        }

    }
}