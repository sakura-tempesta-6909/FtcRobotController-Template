package org.firstinspires.ftc.teamcode.action;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Waits for a specified duration.
 * This will be replaced by Road Runner's SleepAction after integration.
 *
 * 指定された時間待機する。
 * Road Runner導入後は、Road RunnerのSleepActionに置き換えられます。
 */
public class SleepAction implements Action {
    private final double seconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean initialized = false;

    public SleepAction(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public boolean run() {
        if (!initialized) {
            timer.reset();
            initialized = true;
        }
        return timer.seconds() < seconds;
    }
}
