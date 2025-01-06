package org.firstinspires.ftc.teamcode.state;

public class State {
    public enum Mode {
        STOP, //停止中
        DRIVE, //ドライブモード
    }

    // Robot operation mode
    // ロボットの動作モード
    public Mode currentMode;

    public static class DriveState {
        // Current heading of the robot in radians
        // ロボットの現在の方位（ラジアン）
        public double botHeading;
        // Flag to indicate if IMU reset is needed
        // IMUリセットが必要かどうかを示すフラグ
        public boolean imuReset;
        public double xSpeed;
        public double ySpeed;
        public double rotation;

        public boolean charge;
        public boolean discharge;

        public boolean intakeRotation;
        public boolean sliderIsOut;
        public double liftLeftPosition;
        public double liftRightPosition;
        public boolean climb;
        public boolean verticalRotation;
        public double LeftPosition;
        public double RightPosition;

        public double Target;

        public  boolean liftIsDown;

        public boolean sliderIsUp;
        public boolean outtakeCollectorIsOpen;
        public boolean outtakeCollectorIsUP;
        public boolean outtakeCollectorIsRolling;
        public double sliderLeftPosition;
        public double sliderRightPosition;
    }
    public static class SlideState {
        public double currentIntakeVerticalRotationServoPosition = 0.0;
        public double currentLastSliderSlideRightServoPosition = 0.0;
        public double portIntakeVerticalRotationServoNumber = 0;

        public double currentHandRotationServoPosition = 0.0;
        public double portHandRotationServoNumber = 0;
        public boolean verticalRotationUpper = false;

    }


    public static class ControllerState {
        public boolean previousAButtonState; // To track the previous state of gamepad1.a
        public boolean robotCharge; // To toggle the verticalRotation position
        public boolean currentAButtonState;
    }


    // Instances of the sub-classes
    // サブクラスのインスタンス
    public DriveState driveState = new DriveState();
    public SlideState slideState = new SlideState();
    public ControllerState controllerState = new ControllerState();

    public void stateInit() {
        // General
        this.currentMode = Mode.STOP;
        // DriveState
        this.driveState.imuReset = false;
        this.driveState.botHeading = 0.0;
        this.driveState.xSpeed = 0.0;
        this.driveState.ySpeed = 0.0;
        this.driveState.rotation = 0.0;
        this.driveState.intakeRotation = false;
        this.driveState.sliderIsOut = true;
        this.driveState.liftLeftPosition = 0;
        this.driveState.liftRightPosition = 0;
        this.driveState.climb = false;
        this.driveState.verticalRotation = false;
        this.driveState.LeftPosition = 0 ;
        this.driveState.RightPosition = 0 ;
        this.driveState.Target = 0;
        this.driveState.liftIsDown = false;
        this.driveState.sliderIsUp = false;
        this.driveState.outtakeCollectorIsOpen = false;
        this.driveState.outtakeCollectorIsUP = false;
        this.driveState.outtakeCollectorIsRolling = false;
        this.driveState.charge = false;
        this.driveState.discharge = false;
        this.slideState.currentLastSliderSlideRightServoPosition = 0.0;
        this.slideState.currentIntakeVerticalRotationServoPosition  = 0.0;
        this.driveState.sliderLeftPosition = 0;
        this.driveState.sliderRightPosition = 0;
        this.slideState.verticalRotationUpper = false;


        this.controllerState.previousAButtonState =false;
        this.controllerState.robotCharge =false;
        this.controllerState.currentAButtonState = false;
    }

    public void stateReset() {
        // DriveState
        this.driveState.botHeading = 0.0;
        this.driveState.xSpeed = 0.0;
        this.driveState.ySpeed = 0.0;
        this.driveState.rotation = 0.0;

        this.slideState.currentLastSliderSlideRightServoPosition = 0.0;
        this.slideState.currentIntakeVerticalRotationServoPosition  = 0.0;
    }
}