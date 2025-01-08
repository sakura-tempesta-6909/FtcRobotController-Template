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
        public boolean isOuttakeCollectorOpen;
        public boolean isOuttakeCollectorUp;
        public boolean isOuttakeCollectorIsRolling;
    }

    //インテイクの状態
    public enum IntakeMode {
        INIT,
        STOP, // 何もしない
        FINDING,
        CHARGE, // 回収
        DISCHARGE, // 吐き出し
    }

    public enum IntakeOrientation {
        HORIZONTAL, // 横向き
        VERTICAL    // 縦向き
    }

    public static class IntakeState {
        public IntakeMode mode;
        public IntakeOrientation orientation;
    }

    public enum SliderMode {
        DOWN,
        UP
    }

    public static class OuttakeState {
        public boolean outtakeCharge;
        public SliderMode mode;
    }

    // Instances of the subclasses
    // サブクラスのインスタンス
    public DriveState driveState = new DriveState();
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
        this.driveState.isOuttakeCollectorOpen = false;
        this.driveState.isOuttakeCollectorUp = false;
        this.driveState.isOuttakeCollectorIsRolling = false;

        // IntakeState
        this.intakeState.mode = IntakeMode.STOP;
        this.intakeState.orientation = IntakeOrientation.VERTICAL;

        // OuttakeState
        this.outtakeState.outtakeCharge = false;
        this.outtakeState.mode = SliderMode.DOWN;

    }

    public void stateReset() {
        // DriveState
        this.driveState.botHeading = 0.0;
        this.driveState.xSpeed = 0.0;
        this.driveState.ySpeed = 0.0;
        this.driveState.rotation = 0.0;
    }
}