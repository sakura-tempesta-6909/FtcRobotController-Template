package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Robot configuration constants.
 * ロボットの設定定数
 *
 * @Config アノテーションでFTC Dashboardから値を変更可能
 */
@Config
public class RobotConfig {
    private RobotConfig() {} // Prevent instantiation

    // ===================
    // Unit Conversion
    // 単位変換
    // ===================
    // cm to inches: 1 inch = 2.54 cm
    public static final double CM_TO_INCH = 0.3937;  // cm × 0.3937 = inches

    // ===================
    // Drive Motors
    // ===================
    public static final class DriveMotor {
        public static final String LEFT_FRONT = "leftFront";
        public static final String RIGHT_FRONT = "rightFront";
        public static final String LEFT_REAR = "leftBack";
        public static final String RIGHT_REAR = "rightBack";

        public static final DcMotor.Direction LEFT_FRONT_DIR = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction RIGHT_FRONT_DIR = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction LEFT_REAR_DIR = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction RIGHT_REAR_DIR = DcMotor.Direction.REVERSE;
    }

    // ===================
    // IMU
    // ===================
    public static final class Imu {
        public static final String NAME = "imu";
        public static final RevHubOrientationOnRobot ORIENTATION =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                );
    }

    // ===================
    // Vision (AprilTag) - Dashboardから調整可能
    // ===================
    @Config
    public static class Vision {
        public static final String CAMERA_NAME = "Webcam 1";

        // Camera offset from robot center (cm単位)
        // カメラのロボット中心からのオフセット
        // X: 正 = カメラがロボット中心より右、負 = 左
        // Y: 正 = カメラがロボット中心より前
        // Dashboardから調整可能
        public static double CAMERA_X_OFFSET_CM = 0.0;  // 左に10cm
        public static double CAMERA_Y_OFFSET_CM = 19.0;   // 前に17cm

        // Camera heading offset from robot heading (degrees)
        // カメラの向きのロボット向きからのオフセット（度）
        // 0 = カメラが前向き、正 = カメラが右向き
        // Dashboardから調整可能
        public static double CAMERA_HEADING_OFFSET = 0.0;

        // === カメラ露出設定 (Dashboardから調整可能) ===
        // 明るすぎる環境では露出を下げる、暗い環境では上げる
        // AUTO_EXPOSURE = true: 自動露出（通常はこれで良い）
        // AUTO_EXPOSURE = false: 手動露出（MANUAL_EXPOSURE_MSとGAINを使用）
        public static boolean AUTO_EXPOSURE = false;  // 手動露出をデフォルトに
        public static long MANUAL_EXPOSURE_MS = 6;    // 露出時間(ms) 1-100程度、低いほど暗く
        public static int GAIN = 100;                  // ゲイン 0-255、高いほど明るく(ノイズ増)

        // インチに変換（動的に計算）
        public static double getCameraXOffset() {
            return CAMERA_X_OFFSET_CM * CM_TO_INCH;
        }

        public static double getCameraYOffset() {
            return CAMERA_Y_OFFSET_CM * CM_TO_INCH;
        }

        // 後方互換性のため（古いコードがこれを参照している場合）
        public static double CAMERA_X_OFFSET = CAMERA_X_OFFSET_CM * CM_TO_INCH;
        public static double CAMERA_Y_OFFSET = CAMERA_Y_OFFSET_CM * CM_TO_INCH;
    }

    // ===================
    // AprilTag Field Positions - Dashboardから調整可能
    // フィールド上のAprilTagの位置（cm単位で入力）
    // ===================
    @Config
    public static class AprilTag {
        // INTO THE DEEP / DECODE (2024-2025) field tags
        // 座標系: x=左右(正=右), y=前後(正=奥), heading=タグが向いている方向

        // AprilTag size: 16.50 inches (41.91 cm)
        public static final double TAG_SIZE_CM = 41.91;

        // --- GOAL Tags (on field) - Dashboardから調整可能 ---
        // Blue Alliance Goal (DECODE公式データ)
        // フィールド中央を向いている (heading = atan2(141, 148) ≈ 45°)
        public static final int BLUE_GOAL_ID = 20;
        public static double BLUE_GOAL_X_CM = -148.0;
        public static double BLUE_GOAL_Y_CM = -141.0;
        public static double BLUE_GOAL_HEADING = 45.0;

        // Red Alliance Goal (キャリブレーション調整済み)
        // 公式データ(-148, 141)から誤差補正済み
        public static final int RED_GOAL_ID = 24;
        public static double RED_GOAL_X_CM = -128.0;  // -119 - 8.7
        public static double RED_GOAL_Y_CM = 103.0;   // 89 + 14.2
        public static double RED_GOAL_HEADING = -45.0;

        // Helper method to get tag pose in inches
        public static double[] getBlueGoal() {
            return new double[]{BLUE_GOAL_X_CM * CM_TO_INCH, BLUE_GOAL_Y_CM * CM_TO_INCH, BLUE_GOAL_HEADING};
        }

        public static double[] getRedGoal() {
            return new double[]{RED_GOAL_X_CM * CM_TO_INCH, RED_GOAL_Y_CM * CM_TO_INCH, RED_GOAL_HEADING};
        }

        // 全タグのIDと位置のマッピング用
        public static final int[] ALL_TAG_IDS = {
            BLUE_GOAL_ID,
            RED_GOAL_ID
        };

        // 動的に位置を取得するメソッド
        public static double[][] getAllTagPoses() {
            return new double[][]{
                getBlueGoal(),
                getRedGoal()
            };
        }
    }

    // ===================
    // Dead Wheel Configuration
    // デッドホイール設定
    // ===================
    public static final class DeadWheel {
        // Dead wheel diameter: 35mm Dualie Omni Wheel
        // デッドホイール直径: 35mm Dualie Omni Wheel
        public static final double WHEEL_DIAMETER_MM = 35.0;
        public static final double WHEEL_DIAMETER_INCH = WHEEL_DIAMETER_MM / 25.4;

        // Encoder: Rev through-bore (8192 counts/rev)
        // エンコーダー: Rev through-bore (8192カウント/回転)
        public static final int TICKS_PER_REV = 8192;

        // inPerTick = (π × diameter) / ticksPerRev
        public static final double IN_PER_TICK = (Math.PI * WHEEL_DIAMETER_INCH) / TICKS_PER_REV;

        // Parallel encoder position (measures forward/backward)
        // パラレルエンコーダーの位置（前後移動を計測）
        // Y = ロボット中心から左右の距離 (cm, 正 = 左)
        // TODO: 実測値を入力してください
        public static final double PAR_Y_CM = 0.0;

        // Perpendicular encoder position (measures left/right)
        // パーペンディキュラーエンコーダーの位置（左右移動を計測）
        // X = ロボット中心から前後の距離 (cm, 正 = 前)
        public static final double PERP_X_CM = -20.0;  // 後方20cm

        // Convert to tick units for TwoDeadWheelLocalizer
        // TwoDeadWheelLocalizer用にティック単位に変換
        public static final double PAR_Y_TICKS = (PAR_Y_CM * CM_TO_INCH) / IN_PER_TICK;
        public static final double PERP_X_TICKS = (PERP_X_CM * CM_TO_INCH) / IN_PER_TICK;

        // Encoder direction: true = reversed
        // エンコーダー方向: true = 反転
        public static final boolean PAR_REVERSED = false;   // TODO: EncoderDebugで確認後に設定
        public static final boolean PERP_REVERSED = false;  // TODO: EncoderDebugで確認後に設定
    }

    // ===================
    // Control Parameters
    // ===================
    public static final class Control {
        public static final double DEAD_ZONE = 0.1;

        // AprilTag vision correction weight (0.0 = no correction, 1.0 = full correction)
        // AprilTag視覚補正の重み（0.0 = 補正なし、1.0 = 完全補正）
        public static final double VISION_CORRECTION_ALPHA = 0.3;
    }
}