package org.firstinspires.ftc.teamcode.subClass;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.state.State;

public class Util {
    public static void SendLog(State state, Telemetry telemetry) {
        telemetry.addData("CurrentMode", state.currentMode.toString());
    }
}
