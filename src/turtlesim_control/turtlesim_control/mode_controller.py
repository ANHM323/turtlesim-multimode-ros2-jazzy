import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtlesim_interfaces.srv import SetMode, GetMode
from turtlesim_interfaces.action import NavigateWaypoints


class ModeController(Node):
    def __init__(self):
        super().__init__('mode_controller')

        self.mode = 'manual'
        self.circle_dir = 1.0
        self.pose = None

        self.reentrant_group = ReentrantCallbackGroup()
        self.traj_group = MutuallyExclusiveCallbackGroup()

        # Estado de trayectoria
        self.traj_active = False
        self.traj_waypoints_x = []
        self.traj_waypoints_y = []
        self.traj_current_idx = 0
        self.traj_goal_handle = None
        self.traj_lock = threading.Lock()
        self.traj_done_event = threading.Event()

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self._pose_cb, 10, callback_group=self.reentrant_group)

        self.set_mode_srv = self.create_service(
            SetMode, 'set_mode', self._set_mode_cb, callback_group=self.reentrant_group)

        self.get_mode_srv = self.create_service(
            GetMode, 'get_mode', self._get_mode_cb, callback_group=self.reentrant_group)

        self.action_server = ActionServer(
            self, NavigateWaypoints, 'navigate_waypoints',
            execute_callback=self._execute_traj,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.reentrant_group
        )

        self.circle_timer = self.create_timer(0.05, self._circle_loop, callback_group=self.reentrant_group)
        self.circle_timer.cancel()

        self.traj_timer = self.create_timer(0.05, self._traj_control_loop, callback_group=self.traj_group)
        self.traj_timer.cancel()

        self.get_logger().info('Listo. Modo inicial: manual')

    def _pose_cb(self, msg):
        self.pose = msg

    def _get_mode_cb(self, request, response):
        response.success = True
        response.mode = self.mode
        return response

    def _set_mode_cb(self, request, response):
        mode = request.mode.lower()
        valid_modes = ['manual', 'circle_cw', 'circle_ccw', 'trajectory']

        if mode not in valid_modes:
            response.success = False
            response.message = f'Modo invalido. Use: {valid_modes}'
            return response

        self.circle_timer.cancel()
        self.cmd_pub.publish(Twist())
        
        # Si cambian de modo mientras hay trayectoria, abortarla
        with self.traj_lock:
            if self.traj_active and mode != 'trajectory':
                self.traj_active = False
                self.traj_timer.cancel()
                self.traj_done_event.set()

        if mode in ['circle_cw', 'circle_ccw']:
            self.mode = mode
            self.circle_dir = -1.0 if mode == 'circle_cw' else 1.0
            self.circle_timer.reset()
        else:
            self.mode = mode

        response.success = True
        response.message = f'modo={mode}'
        self.get_logger().info(response.message)
        return response

    def _circle_loop(self):
        if not self.mode.startswith('circle'):
            self.circle_timer.cancel()
            return
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 2.0 * self.circle_dir
        self.cmd_pub.publish(cmd)

    def _goal_cb(self, goal_request):
        if self.mode == 'trajectory':
            return GoalResponse.ACCEPT
        self.get_logger().warn('Goal rechazado: primero cambie a modo trajectory')
        return GoalResponse.REJECT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_traj(self, goal_handle):
        xs = goal_handle.request.x
        ys = goal_handle.request.y

        if len(xs) == 0 or len(xs) != len(ys):
            goal_handle.abort()
            return NavigateWaypoints.Result(success=False, message='Waypoints invalidos')

        self.traj_done_event.clear()

        with self.traj_lock:
            self.traj_waypoints_x = list(xs)
            self.traj_waypoints_y = list(ys)
            self.traj_current_idx = 0
            self.traj_goal_handle = goal_handle
            self.traj_active = True
            self.traj_timer.reset()

        self.get_logger().info(f'Iniciando trayectoria con {len(xs)} puntos')

        # Esperar bloqueando solo este hilo, sin interferir con el executor
        self.traj_done_event.wait()

        with self.traj_lock:
            self.traj_timer.cancel()
            self.cmd_pub.publish(Twist())
            
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateWaypoints.Result(success=False, message='Cancelada por usuario')
            elif self.mode != 'trajectory':
                goal_handle.abort()
                result = NavigateWaypoints.Result(success=False, message='Interrumpida por cambio de modo')
            else:
                goal_handle.succeed()
                result = NavigateWaypoints.Result(success=True, message='Trayectoria completada')

            self.traj_goal_handle = None
            self.mode = 'manual'
            self.get_logger().info(result.message)
            return result

    def _traj_control_loop(self):
        with self.traj_lock:
            if not self.traj_active or self.traj_goal_handle is None:
                return
            
            goal_handle = self.traj_goal_handle
            
            if goal_handle.is_cancel_requested:
                self.traj_active = False
                self.traj_done_event.set()
                return

            if self.mode != 'trajectory':
                self.traj_active = False
                self.traj_done_event.set()
                return

            if self.pose is None:
                return

            tx = self.traj_waypoints_x[self.traj_current_idx]
            ty = self.traj_waypoints_y[self.traj_current_idx]

            dx = tx - self.pose.x
            dy = ty - self.pose.y
            dist = math.hypot(dx, dy)

            if dist < 0.15:
                self.traj_current_idx += 1
                if self.traj_current_idx >= len(self.traj_waypoints_x):
                    self.traj_active = False
                    self.traj_done_event.set()
                    return
                return

            theta_des = math.atan2(dy, dx)
            err = theta_des - self.pose.theta
            err = math.atan2(math.sin(err), math.cos(err))

            cmd = Twist()
            cmd.angular.z = max(-2.5, min(2.5, 3.0 * err))
            if abs(err) > 1.0:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = min(2.0, 1.2 * dist)

            self.cmd_pub.publish(cmd)

            feedback = NavigateWaypoints.Feedback()
            feedback.current_index = self.traj_current_idx
            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = ModeController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
