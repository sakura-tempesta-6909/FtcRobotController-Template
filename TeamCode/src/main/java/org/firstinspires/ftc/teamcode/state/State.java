package org.firstinspires.ftc.teamcode.state;

public class State {
    public enum Mode {
        STOP, //停止中
    }

    public Mode currentMode;

    public void stateInit() {
        this.currentMode = Mode.STOP;
    }

    public void stateReset() {

    }
}
