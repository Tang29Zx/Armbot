"""Causal median and One Euro smoothing for numeric state vectors."""

from collections import deque
import math
from statistics import median


class MedianOneEuroFilter:
    """Reject isolated spikes, then adapt smoothing to estimated speed."""

    def __init__(self, dimensions, window_size=3, min_cutoff_hz=1.0,
                 beta=1.5, derivative_cutoff_hz=1.0,
                 reset_gap_sec=0.5):
        if dimensions <= 0:
            raise ValueError('dimensions must be positive')
        if window_size <= 0 or window_size % 2 == 0:
            raise ValueError('window_size must be a positive odd integer')
        if not math.isfinite(min_cutoff_hz) or min_cutoff_hz <= 0.0:
            raise ValueError('min_cutoff_hz must be positive and finite')
        if not math.isfinite(beta) or beta < 0.0:
            raise ValueError('beta must be non-negative and finite')
        if (not math.isfinite(derivative_cutoff_hz)
                or derivative_cutoff_hz <= 0.0):
            raise ValueError(
                'derivative_cutoff_hz must be positive and finite')
        if not math.isfinite(reset_gap_sec) or reset_gap_sec <= 0.0:
            raise ValueError('reset_gap_sec must be positive and finite')
        self._dimensions = dimensions
        self._min_cutoff_hz = min_cutoff_hz
        self._beta = beta
        self._derivative_cutoff_hz = derivative_cutoff_hz
        self._reset_gap_sec = reset_gap_sec
        self._history = [deque(maxlen=window_size)
                         for _ in range(dimensions)]
        self._previous_median = None
        self._filtered = None
        self._filtered_derivative = None
        self._timestamp = None

    def reset(self):
        """Discard all samples and previous One Euro state."""
        for history in self._history:
            history.clear()
        self._previous_median = None
        self._filtered = None
        self._filtered_derivative = None
        self._timestamp = None

    @staticmethod
    def _alpha(cutoff_hz, dt):
        time_constant = 1.0 / (2.0 * math.pi * cutoff_hz)
        return 1.0 / (1.0 + time_constant / dt)

    def update(self, values, timestamp_sec):
        """Return a filtered vector using a finite, increasing timestamp."""
        if len(values) != self._dimensions:
            raise ValueError('value count does not match filter dimensions')
        numeric = [float(value) for value in values]
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError('filter values must be finite')
        timestamp = float(timestamp_sec)
        if not math.isfinite(timestamp):
            raise ValueError('timestamp_sec must be finite')

        if self._timestamp is not None:
            dt = timestamp - self._timestamp
            if dt <= 0.0 or dt > self._reset_gap_sec:
                self.reset()

        for history, value in zip(self._history, numeric):
            history.append(value)
        medians = [median(history) for history in self._history]

        if self._filtered is None:
            self._previous_median = list(medians)
            self._filtered = list(medians)
            self._filtered_derivative = [0.0] * self._dimensions
            self._timestamp = timestamp
            return list(self._filtered)

        dt = timestamp - self._timestamp
        derivative_alpha = self._alpha(self._derivative_cutoff_hz, dt)
        raw_derivative = [
            (current - previous) / dt
            for current, previous in zip(medians, self._previous_median)
        ]
        self._filtered_derivative = [
            previous + derivative_alpha * (current - previous)
            for previous, current in zip(
                self._filtered_derivative, raw_derivative)
        ]
        cutoffs = [
            self._min_cutoff_hz + self._beta * abs(derivative)
            for derivative in self._filtered_derivative
        ]
        self._filtered = [
            previous + self._alpha(cutoff, dt) * (current - previous)
            for previous, current, cutoff in zip(
                self._filtered, medians, cutoffs)
        ]
        self._previous_median = list(medians)
        self._timestamp = timestamp
        return list(self._filtered)
