package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Drive subsystem wrapping Pedro Pathing's Follower.
 * Provides both TeleOp direct control and Autonomous path following.
 *
 * Pedro PathingのFollowerをラップするドライブサブシステム。
 * TeleOpの直接制御とAutonomousのパス追従の両方を提供。
 */
public class Drive {
    // Pedro Follower
    private final Follower follower;

    // Vision (for AprilTag localization)
    private Vision vision;

    public Drive(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose(0, 0, 0));
    }

    public Drive(HardwareMap hardwareMap, Pose initialPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(initialPose);
    }

    /**
     * Set vision subsystem for AprilTag localization.
     * AprilTag位置推定用のVisionサブシステムを設定する。
     */
    public void setVision(Vision vision) {
        this.vision = vision;
    }

    // ===================
    // Pose Management
    // ===================

    /**
     * Get current pose estimate.
     * 現在の位置推定を取得する。
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Set pose estimate.
     * 位置推定を設定する。
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /**
     * Get current heading in radians.
     * 現在の方位をラジアンで取得する。
     */
    public double getHeading() {
        return follower.getPose().getHeading();
    }

    /**
     * Update localization and path following. Call this every loop.
     * 自己位置推定とパス追従を更新する。毎ループ呼び出すこと。
     */
    public void update() {
        follower.update();
    }

    // ===================
    // TeleOp: Direct Control
    // ===================

    /**
     * Start TeleOp drive mode.
     * TeleOpドライブモードを開始する。
     */
    public void startTeleOp() {
        follower.startTeleopDrive();
    }

    /**
     * Drive with field-centric control.
     * フィールドセントリック制御でドライブする。
     *
     * @param forward  Forward speed (-1 to 1, positive = forward)
     * @param strafe   Strafe speed (-1 to 1, positive = left)
     * @param turn     Rotation speed (-1 to 1, positive = counter-clockwise)
     */
    public void drive(double forward, double strafe, double turn) {
        // Pedro uses: forward, strafe, turn, fieldCentric
        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    /**
     * Drive with robot-centric control.
     * ロボットセントリック制御でドライブする。
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(forward, strafe, turn, false);
    }

    /**
     * Stop all motors.
     * 全モーターを停止する。
     */
    public void stop() {
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    // ===================
    // Autonomous: Path Following
    // ===================

    /**
     * Check if a path is currently being followed.
     * パスが現在追従中かどうかを確認する。
     */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /**
     * Follow a path.
     * パスを追従する。
     */
    public void followPath(Path path) {
        follower.followPath(path);
    }

    /**
     * Follow a path chain.
     * パスチェーンを追従する。
     */
    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    /**
     * Follow a path chain with hold end option.
     * 終点保持オプション付きでパスチェーンを追従する。
     */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    /**
     * Get a path builder starting from current pose.
     * 現在位置からのパスビルダーを取得する。
     */
    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    /**
     * Create a simple line path from current pose to target.
     * 現在位置からターゲットへの直線パスを作成する。
     */
    public Path lineTo(Pose target) {
        Path path = new Path(new BezierLine(getPose(), target));
        path.setLinearHeadingInterpolation(getPose().getHeading(), target.getHeading());
        return path;
    }

    /**
     * Create a simple line path with constant heading.
     * 一定のヘディングで直線パスを作成する。
     */
    public Path lineToConstantHeading(Pose target) {
        Path path = new Path(new BezierLine(getPose(), target));
        path.setConstantHeadingInterpolation(getPose().getHeading());
        return path;
    }

    // ===================
    // AprilTag Integration
    // ===================

    /**
     * Correct pose using AprilTag detection.
     * AprilTag検出を使用してposeを補正する。
     *
     * @param tagId Tag ID to look for
     * @return true if correction was applied
     */
    public boolean correctPoseWithTag(int tagId) {
        if (vision == null) return false;

        Pose tagFieldPose = vision.getTagFieldPose(tagId);
        if (tagFieldPose == null) return false;

        AprilTagDetection detection = vision.getDetection(tagId);
        if (detection == null) return false;

        Pose robotPose = vision.calculateRobotPose(detection, tagFieldPose);
        if (robotPose != null) {
            setPose(robotPose);
            return true;
        }

        return false;
    }

    /**
     * Get underlying Follower (for advanced use).
     * 内部のFollowerを取得する（上級者向け）。
     */
    public Follower getFollower() {
        return follower;
    }
}
