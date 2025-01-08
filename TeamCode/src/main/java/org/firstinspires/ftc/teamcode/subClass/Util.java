package org.firstinspires.ftc.teamcode.subClass;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.state.State;

public class Util {
    public static void SendLog(State state, Telemetry telemetry) {
        telemetry.addData("CurrentMode", state.currentMode.toString());
        telemetry.addData("SliderMode", state.outtakeState.mode.toString());
        telemetry.addData("IntakeMode", state.intakeState.mode.toString());
        telemetry.addData("IntakeOrientation", state.intakeState.orientation.toString());
    }


    public static double applyDeadZone(double value) {
        if (Math.abs(value) < Const.Other.CONTROLLER_DEAD_ZONE) {
            return 0.0;
        } else {
            return value;
        }
    }
}