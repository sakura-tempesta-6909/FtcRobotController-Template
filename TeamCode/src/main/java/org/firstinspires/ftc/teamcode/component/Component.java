package org.firstinspires.ftc.teamcode.component;

import org.firstinspires.ftc.teamcode.state.State;

public interface Component {
    /**
     * Read the sensor values.
     * センサーの値を読む。
     */
    void readSensors(State state);

    /**
     * Apply the state.
     * stateを適用する
     */
    void applyState(State state);
}
