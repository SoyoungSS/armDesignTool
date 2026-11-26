"""Audit a task's instantaneous goal predicate on isolated physical states."""
from runtime import mj, np


def check(controller, model, initial, required=False):
    if not hasattr(controller, 'goal_tests') or not hasattr(controller, 'goal_success'):
        if required:
            raise ValueError('Task requires goal_tests(model) and goal_success(model, data)')
        return dict(status='not_provided', reason='No independent goal predicate controls; task evaluator remains task-specific')
    cases = controller.goal_tests(model)
    if not isinstance(cases, list) or not cases:
        raise ValueError('goal_tests must return nonempty positive and negative controls')
    if not all(isinstance(c, dict) and isinstance(c.get('name'), str) and c['name'] and
               type(c.get('expected')) is bool and callable(c.get('setup')) for c in cases):
        raise ValueError('Goal controls require name, boolean expected and setup(model, data)')
    if {c['expected'] for c in cases} != {True, False} or len({c['name'] for c in cases}) != len(cases):
        raise ValueError('Goal controls need unique names and both positive and negative cases')
    spec = mj.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mj.mj_stateSize(model, spec)); mj.mj_getState(model, initial, state, spec)
    reports = []
    for case in cases:
        d = mj.MjData(model); mj.mj_setState(model, d, state, spec)
        case['setup'](model, d); mj.mj_forward(model, d)
        actual = controller.goal_success(model, d)
        if not isinstance(actual, (bool, np.bool_)):
            raise ValueError('goal_success must return a boolean')
        reports.append(dict(name=case['name'], expected=case['expected'], actual=bool(actual),
                            passed=bool(actual) == case['expected']))
    return dict(status='passed' if all(c['passed'] for c in reports) else 'failed', cases=reports,
                scope='goal predicate controls only; not controller/task execution')
