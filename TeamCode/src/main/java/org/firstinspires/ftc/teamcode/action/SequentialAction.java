package org.firstinspires.ftc.teamcode.action;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

/**
 * Executes actions sequentially.
 * This will be replaced by Road Runner's SequentialAction after integration.
 *
 * アクションを順番に実行する。
 * Road Runner導入後は、Road RunnerのSequentialActionに置き換えられます。
 */
public class SequentialAction implements Action {
    private final Queue<Action> actions;

    public SequentialAction(Action... actions) {
        this.actions = new LinkedList<>(Arrays.asList(actions));
    }

    @Override
    public boolean run() {
        if (actions.isEmpty()) {
            return false;
        }

        Action current = actions.peek();
        if (!current.run()) {
            actions.poll(); // Remove completed action
        }

        return !actions.isEmpty();
    }
}
