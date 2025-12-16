package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.geometry.Pose;

/**
 * Drawing utilities for FTC Dashboard field overlay.
 * FTC Dashboardのフィールドオーバーレイ用描画ユーティリティ。
 */
public final class Drawing {
    private Drawing() {}

    private static final double ROBOT_RADIUS = 9;

    /**
     * Draw a robot on the canvas at the given pose.
     * 指定位置にロボットを描画する。
     *
     * @param c    The canvas to draw on
     * @param pose The robot's pose (x, y in inches, heading in radians)
     */
    public static void drawRobot(Canvas c, Pose pose) {
        if (pose == null) return;

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        // Draw robot circle
        c.strokeCircle(x, y, ROBOT_RADIUS);

        // Draw heading direction line
        double halfRadius = 0.5 * ROBOT_RADIUS;
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double x1 = x + halfRadius * cosH;
        double y1 = y + halfRadius * sinH;
        double x2 = x + ROBOT_RADIUS * cosH;
        double y2 = y + ROBOT_RADIUS * sinH;

        c.strokeLine(x1, y1, x2, y2);
    }
}
