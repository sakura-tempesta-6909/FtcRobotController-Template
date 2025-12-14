package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

    // AprilTag field positions (to be configured for your field)
    // フィールド上のAprilTagの位置（フィールドに合わせて設定してください）
    private static final Map<Integer, Pose2d> TAG_FIELD_POSES = new HashMap<>();

    static {
        // Example: INTO THE DEEP field tags (2024-2025 season)
        // 例: INTO THE DEEPフィールドのタグ（2024-2025シーズン）
        // Blue alliance tags
        // TAG_FIELD_POSES.put(1, new Pose2d(60, 36, Math.toRadians(180)));
        // TAG_FIELD_POSES.put(2, new Pose2d(60, 0, Math.toRadians(180)));
        // TAG_FIELD_POSES.put(3, new Pose2d(60, -36, Math.toRadians(180)));
        // Red alliance tags
        // TAG_FIELD_POSES.put(4, new Pose2d(-60, -36, Math.toRadians(0)));
        // TAG_FIELD_POSES.put(5, new Pose2d(-60, 0, Math.toRadians(0)));
        // TAG_FIELD_POSES.put(6, new Pose2d(-60, 36, Math.toRadians(0)));
        // Configure these based on your field setup
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

        // FTC Dashboardにカメラストリームを設定
        dashboard.startCameraStream(this, 30);
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
        return TAG_FIELD_POSES.get(tagId);
    }

    /**
     * Get all configured tag field poses.
     * 設定された全てのタグのフィールド位置を取得する。
     */
    public Map<Integer, Pose2d> getAllTagFieldPoses() {
        return Collections.unmodifiableMap(TAG_FIELD_POSES);
    }

    /**
     * Calculate robot pose from AprilTag detection.
     * AprilTag検出からロボットの位置を計算する。
     *
     * @param detection    The AprilTag detection
     * @param tagFieldPose The tag's position on the field
     * @return Estimated robot pose
     */
    public Pose2d calculateRobotPose(AprilTagDetection detection, Pose2d tagFieldPose) {
        if (detection == null || detection.ftcPose == null || tagFieldPose == null) {
            return null;
        }

        double tagHeading = tagFieldPose.heading.toDouble();

        // Transform camera-relative position to field coordinates
        double robotX = tagFieldPose.position.x
                - detection.ftcPose.y * Math.cos(tagHeading)
                + detection.ftcPose.x * Math.sin(tagHeading);
        double robotY = tagFieldPose.position.y
                - detection.ftcPose.y * Math.sin(tagHeading)
                - detection.ftcPose.x * Math.cos(tagHeading);
        double robotHeading = tagHeading - Math.toRadians(detection.ftcPose.yaw);

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
            Pose2d tagPose = TAG_FIELD_POSES.get(det.id);
            if (tagPose != null && det.ftcPose.range < bestRange) {
                best = det;
                bestRange = det.ftcPose.range;
            }
        }

        if (best == null) {
            return null;
        }

        return calculateRobotPose(best, TAG_FIELD_POSES.get(best.id));
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
