package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.state.State;
import org.firstinspires.ftc.teamcode.subClass.Const;

public class Intake implements Component {
    // サンプルの回収 → intakeCollector
    private final CRServo intakeCollectorLeft;
    private final CRServo intakeCollectorRight;
    // インテイクスライダー → intakeSlider
    public final Servo intakeSliderLeft;
    public final Servo intakeSliderRight;
    // インテイクの横角度(サンプルの向き) → intakeHorizontalRotation
    public final Servo intakeHorizontalRotation;
    // インテイクの縦角度① → intakeVerticalRotation
    public final Servo intakeVerticalRotation;
    // インテイクの縦角度② → intakeLift
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

        intakeVerticalRotation = hardwareMap.get(Servo.class, Const.intake.Name.verticalRotation);
        intakeVerticalRotation.setPosition(Const.intake.Position.verticalRotationInit);
        intakeVerticalRotation.setDirection(Servo.Direction.FORWARD);

        intakeLiftLeft = hardwareMap.get(Servo.class, Const.intake.Name.liftLeft);
        intakeLiftLeft.setDirection(Const.intake.Direction.liftLeft);
        intakeLiftLeft.setPosition(Const.intake.Position.liftInit);

        intakeLiftRight = hardwareMap.get(Servo.class, Const.intake.Name.liftRight);
        intakeLiftRight.setDirection(Const.intake.Direction.liftRight);
        intakeLiftRight.setPosition(Const.intake.Position.liftInit);


        intakeHorizontalRotation.setDirection(Servo.Direction.FORWARD);
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

        switch (state.intakeState.mode) {
            case INIT:
                // 全てが初期位置
                intakeCollectorRight.setPower(Const.intake.Power.collectorPowerInit);
                intakeCollectorLeft.setPower(Const.intake.Power.collectorPowerInit);
                intakeVerticalRotation.setPosition(Const.intake.Position.verticalRotationInit);
                intakeSliderLeft.setPosition(Const.intake.Position.sliderInit);
                intakeSliderRight.setPosition(Const.intake.Position.sliderInit);
                intakeLiftLeft.setPosition(Const.intake.Position.liftInit);
                intakeLiftRight.setPosition(Const.intake.Position.liftInit);
                break;
            case STOP:
                // 回収機構はストップ
                intakeCollectorRight.setPower(Const.intake.Power.collectorPowerInit);
                intakeCollectorLeft.setPower(Const.intake.Power.collectorPowerInit);
                // リフトは前に出ている状態
                intakeLiftLeft.setPosition(Const.intake.Position.liftLimited);
                intakeLiftRight.setPosition(Const.intake.Position.liftLimited);
                // 縦角度①も前に出ている状態
                intakeVerticalRotation.setPosition(Const.intake.Position.verticalRotationSide);
                // スライダーは前に出ている状態
                intakeSliderLeft.setPosition(Const.intake.Position.sliderHead);
                intakeSliderRight.setPosition(Const.intake.Position.sliderHead);
                break;
            case CHARGE:
                // 回収機構は回収方向に回転
                intakeCollectorRight.setPower(Const.intake.Power.ArmPowerCharge);
                intakeCollectorLeft.setPower(Const.intake.Power.ArmPowerCharge);
                // リフトは前に出ている状態に加え、通常よりさらに地面に近く
                intakeLiftLeft.setPosition(Const.intake.Position.liftLowest);
                intakeLiftRight.setPosition(Const.intake.Position.liftLowest);
                // 縦角度①も前に出ている状態
                intakeVerticalRotation.setPosition(Const.intake.Position.verticalRotationSide);
                // スライダーは前に出ている状態
                intakeSliderLeft.setPosition(Const.intake.Position.sliderHead);
                intakeSliderRight.setPosition(Const.intake.Position.sliderHead);
                break;
            case DISCHARGE:
                // 回収機構は排出方向に回転
                intakeCollectorRight.setPower(Const.intake.Power.ArmPowerDischarge);
                intakeCollectorLeft.setPower(Const.intake.Power.ArmPowerDischarge);
                // リフトは前に出ている状態
                intakeLiftLeft.setPosition(Const.intake.Position.liftLimited);
                intakeLiftRight.setPosition(Const.intake.Position.liftLimited);
                // 縦角度①も前に出ている状態
                intakeVerticalRotation.setPosition(Const.intake.Position.verticalRotationSide);
                // スライダーは前に出ている状態
                intakeSliderLeft.setPosition(Const.intake.Position.sliderHead);
                intakeSliderRight.setPosition(Const.intake.Position.sliderHead);
                break;
        }

        // 回収するサンプルの向き
        switch (state.intakeState.orientation) {
            case HORIZONTAL:
                intakeHorizontalRotation.setPosition(Const.intake.Position.horizontalRotationMoving);
                break;
            case VERTICAL:
                intakeHorizontalRotation.setPosition(Const.intake.Position.horizontalRotationInit);
                break;
        }

    }
}