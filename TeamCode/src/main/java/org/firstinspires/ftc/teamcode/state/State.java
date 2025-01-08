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
        public boolean intakeRotation;
        public boolean sliderIsUp;
        public boolean outtakeCollectorIsOpen;
        public boolean outtakeCollectorIsUP;
        public boolean outtakeCollectorIsRolling;
    }
    public static class SlideState {
        public boolean verticalRotationUpper = false;

    }

    public static class IntakeState {
        public boolean charge;
        public boolean discharge;
        public boolean intakeCharge;
    }

    public static class OuttakeState {
        public boolean outtakeCharge;
    }

    // Instances of the subclasses
    // サブクラスのインスタンス
    public DriveState driveState = new DriveState();
    public SlideState slideState = new SlideState();
    public IntakeState intakeState = new IntakeState();
    public OuttakeState outtakeState = new OuttakeState();

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

        // IntakeState
        this.intakeState.charge= false;
        this.intakeState.discharge = false;
        this.intakeState.intakeCharge = false;

        // SliderState
        this.slideState.verticalRotationUpper = false;

        // OuttakeState
        this.outtakeState.outtakeCharge = false;

    }

    public void stateReset() {
        // DriveState
        this.driveState.botHeading = 0.0;
        this.driveState.xSpeed = 0.0;
        this.driveState.ySpeed = 0.0;
        this.driveState.rotation = 0.0;
    }
}