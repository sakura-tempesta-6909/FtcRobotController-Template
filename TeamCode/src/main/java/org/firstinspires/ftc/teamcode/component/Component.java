package org.firstinspires.ftc.teamcode.component;

import org.firstinspires.ftc.teamcode.state.State;

public interface Component {

    /**
     * Initialization for autonomous mode.
     * autonomous時の初期化
     */
    void autonomousInit();

    /**
     * Initialization for teleop mode.
     * teleop時の初期化
     */
    void teleopInit();

    /**
     * Initialization for disabled mode.
     * disable時の初期化
     */
    void disabledInit();

    /**
     * Initialization for test mode.
     * test時の初期化
     */
    void testInit();

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
