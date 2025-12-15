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
 * Vision subsystem for AprilTag detection and robot localization.
 * AprilTag検出とロボット自己位置推定のためのVisionサブシステム。
 *
 * ============================================
 * 座標系 (FTC DECODE 2024-2025)
 * ============================================
 * - X軸: Goal側 (X-) ↔ Audience Wall側 (X+)
 * - Y軸: Red Wall (Y-) ↔ Blue Wall (Y+)
 * - Heading: 0° = X+方向, 90° = Y+方向, 180° = X-方向
 *
 * ============================================
 * AprilTag位置推定の計算フロー
 * ============================================
 * 1. AprilTagを検出し、ftcPose (x, y, yaw) を取得
 *    - ftcPose.x: タグがカメラの右方向にどれだけずれているか (右が正)
 *    - ftcPose.y: タグがカメラの前方向にどれだけ離れているか (前が正)
 *    - ftcPose.yaw: タグがカメラから見てどれだけ回転しているか (右回転が正)
 *
 * 2. カメラのフィールド上の向きを計算
 *    cameraFieldHeading = tagFieldHeading + 180° - yaw
 *    - タグが向いている方向の反対側からカメラが見ている
 *    - yawでカメラの実際の向きを補正
 *
 * 3. タグの相対位置をフィールド座標系に変換
 *    - カメラ座標系 (右がX+, 前がY+) → フィールド座標系
 *    - 回転行列を適用
 *
 * 4. カメラのフィールド位置を計算
 *    cameraFieldPos = tagFieldPos - tagRelativePos
 *
 * 5. ロボット中心の位置を計算
 *    - カメラオフセット (RobotConfig.Vision) を適用
 *    - robotPos = cameraPos - cameraOffset (フィールド座標系で)
 *
 * ============================================
 * 設定値 (RobotConfig.Vision)
 * ============================================
 * - CAMERA_X_OFFSET_CM: カメラのロボット中心からの右方向オフセット (左が負)
 * - CAMERA_Y_OFFSET_CM: カメラのロボット中心からの前方向オフセット
 * - CAMERA_HEADING_OFFSET: カメラの向きのロボット向きからのオフセット (左向きが負)
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
     *
     * 計算の流れ:
     * 1. カメラのフィールド上の向きを計算 (タグの向き + 180° - yaw)
     * 2. タグの相対位置をフィールド座標系に変換
     * 3. カメラのフィールド位置を計算
     * 4. カメラオフセットを適用してロボット中心位置を計算
     *
     * @param detection    The AprilTag detection (ftcPoseが必要)
     * @param tagFieldPose The tag's position on the field (inches, radians)
     * @return Estimated robot pose (inches, radians), or null if invalid input
     */
    public Pose2d calculateRobotPose(AprilTagDetection detection, Pose2d tagFieldPose) {
        if (detection == null || detection.ftcPose == null || tagFieldPose == null) {
            return null;
        }

        // ========== Step 1: カメラのフィールド上の向きを計算 ==========
        // タグがtagHeadingを向いている → カメラはその反対側(+180°)から見ている
        // yaw: タグがカメラから見て右に回転している角度 (正 = 右回転)
        // → カメラはタグ正面から左にずれている → カメラの向きは -yaw 補正
        double tagHeading = tagFieldPose.heading.toDouble();
        double cameraFieldHeading = tagHeading + Math.PI - Math.toRadians(detection.ftcPose.yaw);

        // ========== Step 2: タグの相対位置をフィールド座標系に変換 ==========
        // カメラ座標系: X+ = 右, Y+ = 前
        // フィールド座標系に変換するため、カメラの向きで回転
        double camToTagX = detection.ftcPose.x;  // タグはカメラの右方向に (負なら左)
        double camToTagY = detection.ftcPose.y;  // タグはカメラの前方向に

        double sinH = Math.sin(cameraFieldHeading);
        double cosH = Math.cos(cameraFieldHeading);

        // カメラ→タグのベクトルをフィールド座標系に変換し、符号反転でタグ→カメラに
        // カメラの前方向(Y+) → フィールドの (cosH, sinH) 方向
        // カメラの右方向(X+) → フィールドの (sinH, -cosH) 方向
        double tagToCamera_fieldX = -(camToTagX * sinH + camToTagY * cosH);
        double tagToCamera_fieldY = -(-camToTagX * cosH + camToTagY * sinH);

        // ========== Step 3: カメラのフィールド位置を計算 ==========
        double cameraX = tagFieldPose.position.x + tagToCamera_fieldX;
        double cameraY = tagFieldPose.position.y + tagToCamera_fieldY;

        // ========== Step 4: ロボット中心位置を計算 ==========
        // カメラオフセット (RobotConfig.Vision から取得)
        double cameraXOffset = RobotConfig.Vision.getCameraXOffset();  // 右が正 (inches)
        double cameraYOffset = RobotConfig.Vision.getCameraYOffset();  // 前が正 (inches)
        double cameraHeadingOffset = Math.toRadians(RobotConfig.Vision.CAMERA_HEADING_OFFSET);

        // ロボットの向き = カメラの向き - カメラ取り付け角度オフセット
        double robotHeading = cameraFieldHeading - cameraHeadingOffset;

        // カメラはロボット中心から (Xoffset, Yoffset) の位置にある (ロボット座標系)
        // これをフィールド座標系に変換してカメラ位置から引く
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
