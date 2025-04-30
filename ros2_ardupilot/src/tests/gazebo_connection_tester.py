#!/usr/bin/env python3

import socket
import time

def test_gazebo_connection(host="host.docker.internal", port=11345, timeout=2):
    """Test socket connection to Gazebo on host."""
    try:
        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        
        # Try to connect to Gazebo server
        result = s.connect_ex((host, port))
        
        if result == 0:
            print(f"✅ Successfully connected to Gazebo at {host}:{port}")
            return True
        else:
            print(f"❌ Failed to connect to Gazebo at {host}:{port} (Error code: {result})")
            return False
            
    except Exception as e:
        print(f"❌ Error testing connection: {e}")
        return False
    finally:
        s.close()

# Test common Gazebo ports
for port in [11345, 11346, 5000]:
    test_gazebo_connection(port=port)