package org.firstinspires.ftc.teamcode.opmode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Camera Calibration OpMode.
 * Shows raw AprilTag detection values for camera offset calibration.
 *
 * カメラキャリブレーション用OpMode。
 * カメラオフセットキャリブレーション用のAprilTag検出生値を表示。
 *
 * 使い方:
 * 1. ロボットを既知の位置に置く（例: 0, -65, 90°）
 * 2. カメラをAprilTagに向ける
 * 3. 検出されたx, y, yaw値を記録
 * 4. 期待値との差からカメラオフセットを計算
 */
@Config
@TeleOp(name = "Camera Calibration", group = "Calibration")
public class CameraCalibration extends OpMode implements CameraStreamSource {

    // FTC Dashboardから調整可能（cm単位）
    public static double KNOWN_ROBOT_X_CM = 10;
    public static double KNOWN_ROBOT_Y_CM = -178;
    public static double KNOWN_ROBOT_HEADING_DEG = 90;

    private AprilTagProcessor aprilTag;
    private VisionPortal portal;
    private FtcDashboard dashboard;
    private ExecutorService executor;
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(
            Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        executor = Executors.newSingleThreadExecutor();

        // Dashboard + Driver Hub両方にテレメトリ表示
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.Vision.CAMERA_NAME))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        // FTC Dashboardにカメラストリームを設定
        dashboard.startCameraStream(this, 30);

        telemetry.addLine("=== Camera Calibration ===");
        telemetry.addLine("Point camera at AprilTag");
        telemetry.addData("Known Robot X (cm)", KNOWN_ROBOT_X_CM);
        telemetry.addData("Known Robot Y (cm)", KNOWN_ROBOT_Y_CM);
        telemetry.addData("Known Robot Heading", KNOWN_ROBOT_HEADING_DEG);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> {
            portal.getFrameBitmap(Continuation.create(executor, bitmap -> {
                if (bitmap != null) {
                    lastFrame.set(bitmap);
                }
                bitmapConsumer.accept(lastFrame.get());
            }));
        });
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addLine("=== Camera Calibration ===");
        telemetry.addLine("");
        telemetry.addData("Known Position (cm)", "(%.1f, %.1f, %.1f°)",
                KNOWN_ROBOT_X_CM, KNOWN_ROBOT_Y_CM, KNOWN_ROBOT_HEADING_DEG);
        telemetry.addLine("");

        if (detections.isEmpty()) {
            telemetry.addLine("No AprilTag detected");
            telemetry.addLine("Point camera at a tag");
        } else {
            for (AprilTagDetection det : detections) {
                telemetry.addLine("--- Tag ID: " + det.id + " ---");

                if (det.ftcPose != null) {
                    // Raw detection values (camera-relative)
                    telemetry.addLine("[Raw Detection (camera-relative)]");
                    telemetry.addData("  x (right)", "%.2f inches", det.ftcPose.x);
                    telemetry.addData("  y (forward)", "%.2f inches", det.ftcPose.y);
                    telemetry.addData("  z (up)", "%.2f inches", det.ftcPose.z);
                    telemetry.addData("  yaw", "%.2f deg", det.ftcPose.yaw);
                    telemetry.addData("  pitch", "%.2f deg", det.ftcPose.pitch);
                    telemetry.addData("  roll", "%.2f deg", det.ftcPose.roll);
                    telemetry.addData("  range", "%.2f inches", det.ftcPose.range);
                    telemetry.addData("  bearing", "%.2f deg", det.ftcPose.bearing);
                    telemetry.addData("  elevation", "%.2f deg", det.ftcPose.elevation);
                    telemetry.addLine("");

                    // Field position info (if metadata available)
                    if (det.metadata != null) {
                        telemetry.addLine("[Tag Field Position]");
                        telemetry.addData("  Tag Name", det.metadata.name);
                        telemetry.addData("  Tag Size", "%.2f inches", det.metadata.tagsize);
                    }
                } else {
                    telemetry.addLine("  (pose data not available)");
                }
                telemetry.addLine("");
            }
        }

        telemetry.addLine("=== Instructions ===");
        telemetry.addLine("1. Place robot at known position");
        telemetry.addLine("2. Record x, y, yaw values");
        telemetry.addLine("3. Calculate camera offset:");
        telemetry.addLine("   CAMERA_X_OFFSET = measured offset");
        telemetry.addLine("   CAMERA_Y_OFFSET = measured offset");

        telemetry.update();
    }

    @Override
    public void stop() {
        dashboard.stopCameraStream();
        executor.shutdown();
        if (portal != null) {
            portal.close();
        }
    }
}
