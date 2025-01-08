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
        public boolean sliderIsUp;
        public boolean outtakeCollectorIsOpen;
        public boolean outtakeCollectorIsUP;
        public boolean outtakeCollectorIsRolling;
    }
    public static class SlideState {
        public boolean verticalRotationUpper = false;

    }


    public static class ControllerState {
        public boolean previousGamePad1A; // To track the previous state of gamepad1.a
        public boolean intakeCharge; // To toggle the verticalRotation position
        public boolean currentGamePad1A;

        public boolean previousGamePad2Y; // To track the previous state of gamepad1.a
        public boolean outtakeCharge; // To toggle the verticalRotation position
        public boolean currentGamePad2Y;
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
        this.driveState.sliderIsUp = false;
        this.driveState.outtakeCollectorIsOpen = false;
        this.driveState.outtakeCollectorIsUP = false;
        this.driveState.outtakeCollectorIsRolling = false;
        this.driveState.charge = false;
        this.driveState.discharge = false;
        this.slideState.verticalRotationUpper = false;

        this.controllerState.previousGamePad1A = false;
        this.controllerState.intakeCharge = false;
        this.controllerState.currentGamePad1A = false;

        this.controllerState.previousGamePad2Y = false;
        this.controllerState.outtakeCharge = false;
        this.controllerState.currentGamePad2Y = false;
    }

    public void stateReset() {
        // DriveState
        this.driveState.botHeading = 0.0;
        this.driveState.xSpeed = 0.0;
        this.driveState.ySpeed = 0.0;
        this.driveState.rotation = 0.0;
    }
}