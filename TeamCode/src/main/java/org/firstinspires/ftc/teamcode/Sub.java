package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.component.Slider;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Sub OpMode", group = "Main")
public class Sub extends OpMode {
    private Slider slider;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        slider = new Slider(hardwareMap,telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
    }

    @Override
    public void init_loop() {
        telemetry.addData("SliderCurrentPosition", slider.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
        if (gamepad1.dpad_up) {
            runningActions.add(
                    new SequentialAction(
                            slider.up()
                    )
            );
        } else if (gamepad1.dpad_down) {
            runningActions.add(
                    new SequentialAction(
                            slider.down()
                    )
            );
        }
        telemetry.addData("SliderCurrentPosition", slider.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
