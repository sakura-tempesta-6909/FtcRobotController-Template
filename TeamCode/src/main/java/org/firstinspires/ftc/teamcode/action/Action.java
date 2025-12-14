package org.firstinspires.ftc.teamcode.action;

/**
 * Simple Action interface.
 * This will be replaced by Road Runner's Action after integration.
 *
 * シンプルなActionインターフェース。
 * Road Runner導入後は、Road RunnerのActionに置き換えられます。
 */
public interface Action {
    /**
     * Run the action.
     * @return true if the action should continue, false if complete
     *
     * アクションを実行する。
     * @return 継続する場合はtrue、完了した場合はfalse
     */
    boolean run();
}