#!/usr/bin/env python3
import functools
from typing import Callable, Any
from ardupilot_msgs.msg import Status


class SysIdFilter:
    """Decorator to filter messages by SYSID_THISMAV"""

    def __init__(self, target_sysid: int):
        self.target_sysid = target_sysid

    def __call__(self, callback: Callable) -> Callable:
        @functools.wraps(callback)
        def wrapper(node_self, msg: Any):
            # Check for system_id in the message
            if hasattr(msg, 'system_id'):
                if msg.system_id != self.target_sysid:
                    # Log filtered message for debugging
                    node_self.get_logger().debug(
                        f"Filtering message with system_id={msg.system_id}, expected={self.target_sysid}")
                    return
            # Check for system_id in header if it exists
            elif hasattr(msg, 'header') and hasattr(msg.header, 'system_id'):
                if msg.header.system_id != self.target_sysid:
                    node_self.get_logger().debug(
                        f"Filtering message with header.system_id={msg.header.system_id}, expected={self.target_sysid}")
                    return
            # Fallback: check frame_id for drone identifier
            elif hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
                # Some messages might have drone ID in frame_id
                if f"drone{self.target_sysid}" not in msg.header.frame_id:
                    node_self.get_logger().debug(
                        f"Filtering message with frame_id={msg.header.frame_id}, expected drone{self.target_sysid}")
                    return
            # If no filtering criteria found, pass through with warning
            else:
                node_self.get_logger().debug(
                    f"Message type {type(msg).__name__} has no system_id filtering - passing through")
            
            return callback(node_self, msg)

        return wrapper