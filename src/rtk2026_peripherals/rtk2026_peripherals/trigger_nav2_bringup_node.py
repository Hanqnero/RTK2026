# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Waits delay_sec, then until TF (source_frame -> target_frame) is available,
# then calls lifecycle_manager manage_nodes STARTUP (for use_fake_odom when
# Nav2 is started with autostart:=false). No hardcoded frame IDs.

import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import Buffer, TransformListener


def main(args=None):
    rclpy.init(args=args)
    node = Node("trigger_nav2_bringup")
    node.declare_parameter("delay_sec", 15)
    node.declare_parameter("service_name", "/lifecycle_manager_navigation/manage_nodes")
    node.declare_parameter("wait_for_node", "controller_server")
    node.declare_parameter("target_frame", "odom")
    node.declare_parameter("source_frame", "base_link")
    node.declare_parameter("tf_timeout_sec", 120)
    node.declare_parameter("lifecycle_wait_sec", 60)

    def _int_param(name: str, default: int) -> int:
        p = node.get_parameter(name).get_parameter_value()
        if p.type == ParameterType.PARAMETER_INTEGER:
            return p.integer_value
        if p.type == ParameterType.PARAMETER_STRING:
            return int(p.string_value)
        return default

    delay = _int_param("delay_sec", 15)
    srv_name = node.get_parameter("service_name").get_parameter_value().string_value
    wait_node = node.get_parameter("wait_for_node").get_parameter_value().string_value
    target_frame = node.get_parameter("target_frame").get_parameter_value().string_value
    source_frame = node.get_parameter("source_frame").get_parameter_value().string_value
    tf_timeout = _int_param("tf_timeout_sec", 120)
    lifecycle_wait = _int_param("lifecycle_wait_sec", 60)

    node.get_logger().info(
        "Waiting %ds then TF %s -> %s (timeout %ds) before Nav2 manage_nodes STARTUP"
        % (delay, source_frame, target_frame, tf_timeout)
    )
    time.sleep(max(0, delay))

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    deadline = time.monotonic() + tf_timeout
    while rclpy.ok():
        if time.monotonic() > deadline:
            node.get_logger().warn(
                "TF %s -> %s not available after %ds, calling manage_nodes anyway"
                % (source_frame, target_frame, tf_timeout)
            )
            break
        try:
            if tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):
                node.get_logger().info("TF %s -> %s is available" % (source_frame, target_frame))
                break
        except Exception:
            pass
        rclpy.spin_once(node, timeout_sec=0.5)
    # No explicit destroy needed for TransformListener; it will be cleaned up with the node.

    change_state_srv = "/%s/change_state" % wait_node
    probe = node.create_client(ChangeState, change_state_srv)
    if not probe.wait_for_service(timeout_sec=float(lifecycle_wait)):
        node.get_logger().warn(
            "Service %s not available after %ds, calling manage_nodes anyway" % (change_state_srv, lifecycle_wait)
        )
    else:
        node.get_logger().info("Lifecycle node %s is ready" % wait_node)
    probe.destroy()

    client = node.create_client(ManageLifecycleNodes, srv_name)
    if not client.wait_for_service(timeout_sec=30.0):
        node.get_logger().error("Service %s not available" % srv_name)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return 1

    req = ManageLifecycleNodes.Request()
    req.command = 0  # STARTUP
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=600.0)
    if future.result() is not None and future.result().success:
        node.get_logger().info("Nav2 manage_nodes STARTUP succeeded")
    else:
        node.get_logger().error("Nav2 manage_nodes STARTUP failed or timed out")
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    exit(main())
