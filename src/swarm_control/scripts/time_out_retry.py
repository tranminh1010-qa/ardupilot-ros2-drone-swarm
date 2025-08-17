#!/usr/bin/env python3
import functools
import time
import rclpy
from typing import Optional, TypeVar, Callable, Any

T = TypeVar('T')


class TimeoutRetry:
    """Decorator for operations with timeout and retry logic"""

    def __init__(self, timeout_sec: float, retry_interval: float = 1.0,
                 success_check_factory: Optional[Callable] = None):
        self.timeout_sec = timeout_sec
        self.retry_interval = retry_interval
        self.success_check_factory = success_check_factory

    def __call__(self, func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(node_self, *args, **kwargs):
            # Get success check based on function and arguments
            if self.success_check_factory:
                success_check = self.success_check_factory(*args, **kwargs)
            else:
                success_check = lambda x: bool(x)

            start = node_self.get_clock().now()
            timeout_duration = rclpy.duration.Duration(seconds=self.timeout_sec)

            while node_self.get_clock().now() - start < timeout_duration:
                try:
                    result = func(node_self, *args, **kwargs)
                    if success_check(result):
                        return result
                except Exception as e:
                    node_self.get_logger().warning(f"Retry: {e}")

                time.sleep(self.retry_interval)
                rclpy.spin_once(node_self, timeout_sec=0.1)

            raise TimeoutError(f"Operation timed out")

        return wrapper