package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.lib.Drawing;
import org.firstinspires.ftc.teamcode.subsystem.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Camera Calibration OpMode for adjusting camera position offsets.
 * カメラ位置オフセットを調整するためのキャリブレーションOpMode。
 *
 * 使い方:
 * 1. ロボットを既知の位置に置く
 * 2. Dashboardで KNOWN_ROBOT_X_CM, Y_CM, HEADING_DEG を設定
 * 3. AprilTagが見える位置にカメラを向ける
 * 4. Dashboardで RobotConfig$Vision の値を調整
 * 5. 計算されたロボット位置(緑)が既知の位置(青)と一致するまで調整
 *
 * ボタン:
 * - X: 露出設定を適用
 * - A: 現在の計算位置を既知位置として設定
 */
@Config
@TeleOp(name = "Camera Calibration", group = "Calibration")
public class CameraCalibration extends OpMode {

    // FTC Dashboardから調整可能（cm単位）
    // FTC DECODE座標系: X+ = Audience Wall, X- = Goal, Y+ = Blue Wall, Y- = Red Wall
    public static double KNOWN_ROBOT_X_CM = 152.0;
    public static double KNOWN_ROBOT_Y_CM = 33.0;
    public static double KNOWN_ROBOT_HEADING_DEG = 145.0;

    private Vision vision;
    private FtcDashboard dashboard;
    private boolean lastX = false;
    private boolean lastA = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Visionサブシステムを初期化（計算ロジックはVisionクラスに集約）
        vision = new Vision(hardwareMap);

        telemetry.addLine("=== Camera Calibration ===");
        telemetry.addLine("X: Apply exposure | A: Set current as known");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ボタン処理
        if (gamepad1.x && !lastX) {
            vision.applyExposureSettings();
        }
        lastX = gamepad1.x;

        if (gamepad1.a && !lastA) {
            Pose calculated = vision.getBestPoseEstimate();
            if (calculated != null) {
                KNOWN_ROBOT_X_CM = calculated.getX() / RobotConfig.CM_TO_INCH;
                KNOWN_ROBOT_Y_CM = calculated.getY() / RobotConfig.CM_TO_INCH;
                KNOWN_ROBOT_HEADING_DEG = Math.toDegrees(calculated.getHeading());
            }
        }
        lastA = gamepad1.a;

        // Dashboard用パケット
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // 既知の位置を青で表示
        Pose knownPose = new Pose(
                KNOWN_ROBOT_X_CM * RobotConfig.CM_TO_INCH,
                KNOWN_ROBOT_Y_CM * RobotConfig.CM_TO_INCH,
                Math.toRadians(KNOWN_ROBOT_HEADING_DEG)
        );
        canvas.setStroke("#3F51B5");  // Blue
        canvas.setStrokeWidth(2);
        Drawing.drawRobot(canvas, knownPose);

        // カメラオフセット設定
        telemetry.addLine("--- Camera Offset ---");
        telemetry.addData("X", "%.1f cm", RobotConfig.Vision.CAMERA_X_OFFSET_CM);
        telemetry.addData("Y", "%.1f cm", RobotConfig.Vision.CAMERA_Y_OFFSET_CM);
        telemetry.addData("Heading", "%.1f deg", RobotConfig.Vision.CAMERA_HEADING_OFFSET);

        // 既知の位置
        telemetry.addLine("--- Known Position (Blue) ---");
        telemetry.addData("Position", "(%.1f, %.1f) cm", KNOWN_ROBOT_X_CM, KNOWN_ROBOT_Y_CM);
        telemetry.addData("Heading", "%.1f deg", KNOWN_ROBOT_HEADING_DEG);

        // AprilTag検出
        List<AprilTagDetection> detections = vision.getAllDetections();

        if (detections.isEmpty()) {
            telemetry.addLine("--- No AprilTag Detected ---");
        } else {
            AprilTagDetection det = detections.get(0);
            Pose tagFieldPose = vision.getTagFieldPose(det.id);

            // 検出情報
            telemetry.addLine("--- Detection (Tag " + det.id + ") ---");
            telemetry.addData("Range", "%.1f cm", det.ftcPose.range / RobotConfig.CM_TO_INCH);
            telemetry.addData("Yaw", "%.1f deg", det.ftcPose.yaw);

            // Visionクラスで計算
            Pose calculatedPose = vision.calculateRobotPose(det, tagFieldPose);

            if (calculatedPose != null) {
                // 計算された位置を緑で表示
                canvas.setStroke("#4CAF50");  // Green
                canvas.setStrokeWidth(2);
                Drawing.drawRobot(canvas, calculatedPose);

                double calcXCm = calculatedPose.getX() / RobotConfig.CM_TO_INCH;
                double calcYCm = calculatedPose.getY() / RobotConfig.CM_TO_INCH;
                double calcHeadingDeg = Math.toDegrees(calculatedPose.getHeading());

                telemetry.addLine("--- Calculated Position (Green) ---");
                telemetry.addData("Position", "(%.1f, %.1f) cm", calcXCm, calcYCm);
                telemetry.addData("Heading", "%.1f deg", calcHeadingDeg);

                // 誤差
                double dxCm = calcXCm - KNOWN_ROBOT_X_CM;
                double dyCm = calcYCm - KNOWN_ROBOT_Y_CM;
                double dHeading = calcHeadingDeg - KNOWN_ROBOT_HEADING_DEG;
                while (dHeading > 180) dHeading -= 360;
                while (dHeading < -180) dHeading += 360;
                double distCm = Math.sqrt(dxCm * dxCm + dyCm * dyCm);

                telemetry.addLine("--- Error ---");
                telemetry.addData("Delta", "(%.1f, %.1f) cm, %.1f deg", dxCm, dyCm, dHeading);
                telemetry.addData("Distance", "%.1f cm", distCm);

                // Dashboard用
                packet.put("Error X (cm)", dxCm);
                packet.put("Error Y (cm)", dyCm);
                packet.put("Error Heading (deg)", dHeading);
                packet.put("Error Distance (cm)", distCm);
            }
        }

        telemetry.addLine("--- Controls ---");
        telemetry.addData("X", "Apply exposure (" + vision.getExposureInfo() + ")");
        telemetry.addData("A", "Set current as known position");

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.close();
        }
    }
}
