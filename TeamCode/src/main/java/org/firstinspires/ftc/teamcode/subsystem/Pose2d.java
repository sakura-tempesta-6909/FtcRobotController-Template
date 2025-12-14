package org.firstinspires.ftc.teamcode.subsystem;

/**
 * Simple 2D pose representation.
 * This will be replaced by Road Runner's Pose2d after integration.
 *
 * シンプルな2次元位置表現。
 * Road Runner導入後は、Road RunnerのPose2dに置き換えられます。
 */
public class Pose2d {
    public final double x;
    public final double y;
    public final double heading; // radians

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d() {
        this(0, 0, 0);
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.2f°)", x, y, Math.toDegrees(heading));
    }
}
