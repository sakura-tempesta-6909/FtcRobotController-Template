package org.firstinspires.ftc.teamcode.subClass;

import android.util.Log;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A/B相の DigitalChannel からティック数を自前でカウントするクラス
 */
public class QuadratureEncoder {
    private final DigitalChannel chanA, chanB;
    private int lastState;
    private int count = 0;

    /**
     * 4相遷移ごとのインクリメントテーブル
     */
    private static final int[][] TRANSITION = {
            /* to 00 */ {0, -1, +1, 0},
            /* to 01 */ {+1, 0, 0, -1},
            /* to 10 */ {-1, 0, 0, +1},
            /* to 11 */ {0, +1, -1, 0},
    };

    /**
     * nameA＝A相ポート名, nameB＝B相ポート名
     */
    public QuadratureEncoder(HardwareMap map, String nameA, String nameB) {
        chanA = map.get(DigitalChannel.class, nameA);
        chanB = map.get(DigitalChannel.class, nameB);
        chanA.setMode(DigitalChannel.Mode.INPUT);
        chanB.setMode(DigitalChannel.Mode.INPUT);
        // 初期状態を取っておく
        int a = chanA.getState() ? 1 : 0;
        int b = chanB.getState() ? 1 : 0;
        lastState = (a << 1) | b;
    }

    /**
     * 毎サイクル呼び出してカウントを更新
     */
    public void update() {
        int a = chanA.getState() ? 1 : 0;
        int b = chanB.getState() ? 1 : 0;
        int state = (a << 1) | b;
        // 前回状態→今回状態 の差分をテーブル参照
        count += TRANSITION[lastState][state];
        lastState = state;
        Log.i("Encoder: chanA", chanA.getState() + "");
        Log.i("Encoder: chanB", chanB.getState() + "");
    }

    /**
     * 現在のカウント（ティック数）を取得
     */
    public int getCurrentPosition() {
        return count;
    }

    /**
     * カウントをリセット
     */
    public void reset() {
        count = 0;
        // 基準状態も更新しておく
        int a = chanA.getState() ? 1 : 0;
        int b = chanB.getState() ? 1 : 0;
        lastState = (a << 1) | b;
    }
}
