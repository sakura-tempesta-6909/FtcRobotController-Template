package org.firstinspires.ftc.teamcode.action;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Executes actions in parallel.
 * This will be replaced by Road Runner's ParallelAction after integration.
 *
 * アクションを並列に実行する。
 * Road Runner導入後は、Road RunnerのParallelActionに置き換えられます。
 */
public class ParallelAction implements Action {
    private final List<Action> actions;

    public ParallelAction(Action... actions) {
        this.actions = new ArrayList<>(Arrays.asList(actions));
    }

    @Override
    public boolean run() {
        actions.removeIf(action -> !action.run());
        return !actions.isEmpty();
    }
}
