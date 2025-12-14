package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

/**
 * Vision subsystem for AprilTag detection and localization.
 * Supports camera preview on FTC Dashboard and Driver Hub.
 *
 * AprilTag検出と自己位置推定のためのVisionサブシステム。
 * FTCダッシュボードとDriver Hubへのカメラプレビュー表示をサポート。
 */
public class Vision implements CameraStreamSource {
    private final AprilTagProcessor aprilTag;
    private final VisionPortal portal;
    private final FtcDashboard dashboard;
    private final ExecutorService executor;

    // For dashboard streaming
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    /**
     * Get tag field pose dynamically (supports Dashboard updates).
     * タグのフィールド位置を動的に取得（Dashboard更新をサポート）
     */
    private Pose2d getTagFieldPoseDynamic(int tagId) {
        double[] pose = null;
        if (tagId == RobotConfig.AprilTag.BLUE_GOAL_ID) {
            pose = RobotConfig.AprilTag.getBlueGoal();
        } else if (tagId == RobotConfig.AprilTag.RED_GOAL_ID) {
            pose = RobotConfig.AprilTag.getRedGoal();
        }

        if (pose != null && pose.length >= 3) {
            return new Pose2d(pose[0], pose[1], Math.toRadians(pose[2]));
        }
        return null;
    }

    public Vision(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();
        executor = Executors.newSingleThreadExecutor();

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.Vision.CAMERA_NAME))
                .addProcessor(aprilTag)
                // Driver Hubにプレビュー表示
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        // カメラが準備できるまで待機してから露出設定を適用
        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // Wait for camera to start streaming
        }
        applyExposureSettings();

        // FTC Dashboardにカメラストリームを設定
        dashboard.startCameraStream(this, 30);
    }

    /**
     * Apply exposure settings from RobotConfig.
     * RobotConfigから露出設定を適用する。
     * Dashboardから値を変更した後にこのメソッドを呼び出すと反映される。
     */
    public void applyExposureSettings() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);

        if (exposureControl != null) {
            if (RobotConfig.Vision.AUTO_EXPOSURE) {
                exposureControl.setMode(ExposureControl.Mode.Auto);
            } else {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(RobotConfig.Vision.MANUAL_EXPOSURE_MS, TimeUnit.MILLISECONDS);
            }
        }

        if (gainControl != null && !RobotConfig.Vision.AUTO_EXPOSURE) {
            gainControl.setGain(RobotConfig.Vision.GAIN);
        }
    }

    /**
     * Get current exposure info for debugging.
     * デバッグ用の現在の露出情報を取得する。
     */
    public String getExposureInfo() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return "Camera not streaming";
        }

        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);

        StringBuilder info = new StringBuilder();
        if (exposureControl != null) {
            info.append("Exposure: ").append(exposureControl.getExposure(TimeUnit.MILLISECONDS)).append("ms");
            info.append(" (").append(exposureControl.getMode()).append(")");
        }
        if (gainControl != null) {
            info.append(", Gain: ").append(gainControl.getGain());
        }
        return info.toString();
    }

    /**
     * CameraStreamSource implementation for FTC Dashboard.
     * FTCダッシュボード用のカメラストリーム実装。
     */
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> {
            // Get frame from VisionPortal
            portal.getFrameBitmap(Continuation.create(executor, bitmap -> {
                if (bitmap != null) {
                    lastFrame.set(bitmap);
                }
                bitmapConsumer.accept(lastFrame.get());
            }));
        });
    }

    /**
     * Get detection for a specific tag ID.
     * 指定されたタグIDの検出結果を取得する。
     *
     * @param tagId The AprilTag ID
     * @return Detection if found, null otherwise
     */
    public AprilTagDetection getDetection(int tagId) {
        for (AprilTagDetection det : aprilTag.getDetections()) {
            if (det.id == tagId && det.ftcPose != null) {
                return det;
            }
        }
        return null;
    }

    /**
     * Get all current detections.
     * 現在の全ての検出結果を取得する。
     */
    public List<AprilTagDetection> getAllDetections() {
        return aprilTag.getDetections().stream()
                .filter(d -> d.ftcPose != null)
                .collect(Collectors.toList());
    }

    /**
     * Get field pose for a tag ID.
     * タグIDに対応するフィールド上の位置を取得する。
     */
    public Pose2d getTagFieldPose(int tagId) {
        return getTagFieldPoseDynamic(tagId);
    }

    /**
     * Get all configured tag field poses.
     * 設定された全てのタグのフィールド位置を取得する。
     */
    public Map<Integer, Pose2d> getAllTagFieldPoses() {
        Map<Integer, Pose2d> poses = new HashMap<>();
        poses.put(RobotConfig.AprilTag.BLUE_GOAL_ID, getTagFieldPoseDynamic(RobotConfig.AprilTag.BLUE_GOAL_ID));
        poses.put(RobotConfig.AprilTag.RED_GOAL_ID, getTagFieldPoseDynamic(RobotConfig.AprilTag.RED_GOAL_ID));
        return poses;
    }

    /**
     * Calculate robot pose from AprilTag detection.
     * AprilTag検出からロボットの位置を計算する。
     * カメラオフセット（RobotConfig.Vision）を考慮。
     *
     * @param detection    The AprilTag detection
     * @param tagFieldPose The tag's position on the field
     * @return Estimated robot pose
     */
    public Pose2d calculateRobotPose(AprilTagDetection detection, Pose2d tagFieldPose) {
        if (detection == null || detection.ftcPose == null || tagFieldPose == null) {
            return null;
        }

        // タグの向き（フィールド座標系）
        double tagHeading = tagFieldPose.heading.toDouble();

        // Camera offset from RobotConfig (Dashboardから動的に取得)
        double cameraXOffset = RobotConfig.Vision.getCameraXOffset();
        double cameraYOffset = RobotConfig.Vision.getCameraYOffset();
        double cameraHeadingOffset = Math.toRadians(RobotConfig.Vision.CAMERA_HEADING_OFFSET);

        // カメラのフィールド上の向きを計算
        // タグがtagHeadingを向いている時、カメラはその反対側(+π)から見ている
        // yawはカメラから見たタグの回転角度（正 = タグが反時計回りに見える = カメラが時計回りにずれている）
        double cameraFieldHeading = tagHeading + Math.PI + Math.toRadians(detection.ftcPose.yaw);

        // カメラから見たタグの位置 (ftcPose.x = 右方向, ftcPose.y = 前方向)
        double camToTagX = detection.ftcPose.x;  // カメラの右方向
        double camToTagY = detection.ftcPose.y;  // カメラの前方向

        // カメラ座標系からフィールド座標系への変換
        // カメラの前方向(+Y)はcameraFieldHeading、右方向(+X)はcameraFieldHeading - 90°
        // camToTag_field = rotate(camToTag_camera, cameraFieldHeading)
        // tagToCamera_field = -camToTag_field
        double sinH = Math.sin(cameraFieldHeading);
        double cosH = Math.cos(cameraFieldHeading);
        double tagToCamera_fieldX = -(camToTagX * sinH + camToTagY * cosH);
        double tagToCamera_fieldY = -(-camToTagX * cosH + camToTagY * sinH);

        // カメラのフィールド位置
        double cameraX = tagFieldPose.position.x + tagToCamera_fieldX;
        double cameraY = tagFieldPose.position.y + tagToCamera_fieldY;

        // ロボットの向き（カメラの向き - カメラオフセット）
        double robotHeading = cameraFieldHeading - cameraHeadingOffset;

        // カメラ位置からロボット中心位置を計算
        // カメラはロボット中心から (cameraXOffset = 右, cameraYOffset = 前) の位置にある
        // ロボット座標系での右方向 = robotHeading - 90° = sin(robotHeading) in field X
        // ロボット座標系での前方向 = robotHeading = cos(robotHeading) in field X
        double sinR = Math.sin(robotHeading);
        double cosR = Math.cos(robotHeading);
        double robotX = cameraX - (cameraXOffset * sinR + cameraYOffset * cosR);
        double robotY = cameraY - (-cameraXOffset * cosR + cameraYOffset * sinR);

        return new Pose2d(robotX, robotY, robotHeading);
    }

    /**
     * Get best pose estimate from all visible tags.
     * 見えている全てのタグから最良の位置推定を取得する。
     *
     * @return Best pose estimate, or null if no tags visible
     */
    public Pose2d getBestPoseEstimate() {
        List<AprilTagDetection> detections = getAllDetections();

        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection det : detections) {
            Pose2d tagPose = getTagFieldPoseDynamic(det.id);
            if (tagPose != null && det.ftcPose.range < bestRange) {
                best = det;
                bestRange = det.ftcPose.range;
            }
        }

        if (best == null) {
            return null;
        }

        return calculateRobotPose(best, getTagFieldPoseDynamic(best.id));
    }

    /**
     * Check if any AprilTag is currently visible.
     * AprilTagが現在見えているかどうかを確認する。
     */
    public boolean isTagVisible() {
        return !getAllDetections().isEmpty();
    }

    /**
     * Check if a specific AprilTag is visible.
     * 特定のAprilTagが見えているかどうかを確認する。
     */
    public boolean isTagVisible(int tagId) {
        return getDetection(tagId) != null;
    }

    /**
     * Close vision portal and stop dashboard stream.
     * VisionPortalを閉じ、ダッシュボードストリームを停止する。
     */
    public void close() {
        dashboard.stopCameraStream();
        executor.shutdown();
        portal.close();
    }
}
