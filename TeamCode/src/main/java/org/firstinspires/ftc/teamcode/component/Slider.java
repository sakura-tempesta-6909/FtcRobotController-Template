package org.firstinspires.ftc.teamcode.component;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slider {
    private final DcMotorEx motor;
    private final Telemetry telemetry;

    // 定数
    private static final int UP_POSITION = 1700;
    private static final int DOWN_POSITION = 0;
    private static final double MOVE_POWER = 0.7;
    private static final int TOLERANCE = 10;

    public Slider(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "slider");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(DOWN_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.telemetry =  telemetry;
    }

    /** スライダーを上げる Action */
    public class SlideUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                motor.setTargetPosition(UP_POSITION);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // 安全のため再設定
                motor.setPower(MOVE_POWER);
            }

            int current = motor.getCurrentPosition();
            int error = UP_POSITION - current;
            boolean atTarget = Math.abs(error) <= TOLERANCE;

            packet.put("SliderPos", current);
            packet.put("TargetPos", UP_POSITION);
            packet.put("Error", error);
            packet.put("IsBusy", motor.isBusy());
            packet.put("Mode", motor.getMode().name());

            if (atTarget) {
                motor.setPower(0);
            }

            return atTarget;
        }
    }

    /** スライダーを下げる Action */
    public class SlideDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                motor.setTargetPosition(DOWN_POSITION);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(MOVE_POWER);
                telemetry.addData("SliderPos", motor.getCurrentPosition());
                telemetry.update();
            }

            int current = motor.getCurrentPosition();
            int error = DOWN_POSITION - current;
            boolean atTarget = Math.abs(error) <= TOLERANCE;

            packet.put("SliderPos", current);
            packet.put("TargetPos", DOWN_POSITION);
            packet.put("Error", error);
            packet.put("IsBusy", motor.isBusy());
            packet.put("Mode", motor.getMode().name());

            if (atTarget) {
                motor.setPower(0);
            }

            return atTarget;
        }
    }

    public Action up() {
        return new SlideUp();
    }

    public Action down() {
        return new SlideDown();
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
