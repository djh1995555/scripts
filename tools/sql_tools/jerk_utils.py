from functools import reduce
import numpy as np
import operator as op


class TrackingDifferentiator(object):
    """this class is used to filter data, and extract derivatives without too much noise

    take a transfer function as this:
        y(s) / u(s) = r^n / (s + r)^n
    where u represents our raw input, y represents output, r denotes the tracking factor,
    n denotes the order, and s is the Laplace operator.

    convert this into a state-space expression, and we have x_1 = y, x_2 = dot{x_1},
    x_3 = dot{x_2}, ... Then if tracking factor is great enough, x_1 or y will well track the
    raw input, and x_2, x_3, .... will approximate the first, second, third, .... derivatives
    of the raw input. By tuning the tracking factor, we can balance the noise reduction and
    tracking performance. Large tracking factor means the output can fastly catch up with the
    raw input and data noise is preserved. In contrast, small factor means less noise but
    may track poorly.

    """

    def __init__(self, tracking_factor=10, order=2):
        self._tracking_factor = tracking_factor
        self._order = order

        self._state_step_matrix = np.diag(np.ones(order - 1), 1)
        self._state_step_matrix[-1, :] = [
            -get_combination_num(order, order - i)
            * self._tracking_factor ** (order - i)
            for i in range(order)
        ]
        self._input_step_vector = np.zeros((order, 1))
        self._input_step_vector[-1, :] = self._tracking_factor ** order

        self._last_timestamp = None
        self.state = None
        self.is_activated = False

    def reset(self, reset_timestamp, reset_state):
        self._last_timestamp = reset_timestamp
        self.state = self._check_state_dims(reset_state, self._order)
        self.is_activated = True

    def step(self, new_input, timestamp):
        dot_state = (
            np.matmul(self._state_step_matrix, self.state)
            + self._input_step_vector * new_input
        )
        self.state += dot_state * (timestamp - self._last_timestamp)
        self._last_timestamp = timestamp
        return self.state[0, 0], self.state[1, 0]

    def _check_state_dims(self, state, dim):
        if dim < 2:
            raise ValueError("dim of tracking differentiator should be >= 2. ")
        if isinstance(state, list):
            if len(state) != dim:
                raise ValueError("tracking differentiator dims mismatch. ")
            state = np.array(state, dtype=np.float64).reshape(-1, 1)
        elif isinstance(state, np.array):
            if state.size != dim:
                raise ValueError("tracking differentiator dims mismatch. ")
            state = state.reshape(-1, 1)
        else:
            raise ValueError("not supported state. ")
        return state


def get_combination_num(n, r):
    """get number of combinations, from n choose r"""
    r = min(r, n - r)
    numer = reduce(op.mul, range(n, n - r, -1), 1)
    denom = reduce(op.mul, range(1, r + 1), 1)
    return numer // denom
