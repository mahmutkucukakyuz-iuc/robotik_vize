"""Ortak yardimcilar: FollowJointTrajectory istemcisi ve arka plan spin."""
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'


class _QuietShutdownExecutor(MultiThreadedExecutor):
    """Lyrical rclpy'de MultiThreadedExecutor.shutdown() guard condition'i, calismakta olan
    callback'ler bitmeden yok ediyor; o callback'ler sonunda 'The following exception was never
    retrieved: cannot use Destroyable because destruction was requested' yazdiriyor (zararsiz ama
    ekranda gorunuyor). Once node'lar cikarilir (yeni callback dagitilmaz), calisanlarin bitmesi
    kisa bir sure beklenir, sonra normal shutdown yapilir."""

    def shutdown(self, timeout_sec=None, **kwargs):
        try:
            if rclpy.ok():
                for n in list(self.get_nodes()):
                    self.remove_node(n)
                time.sleep(0.2)
        except Exception:  # noqa: BLE001
            pass
        return super().shutdown(timeout_sec, **kwargs)


def start_spin_thread(node):
    """Node'u arka plandaki bir executor'da dondurur; ana thread serbest kalir."""
    executor = _QuietShutdownExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception:  # noqa: BLE001
            # Ctrl+C sonrasi context kapaninca rclpy RCLError atar; bu normal kapanistir
            if rclpy.ok():
                raise

    th = threading.Thread(target=spin, daemon=True)
    th.start()
    return executor, th


def wait_future(future, timeout_s):
    """Future tamamlanana kadar (arka plan executor'u ile) bekler."""
    t0 = time.monotonic()
    while rclpy.ok() and not future.done():
        if time.monotonic() - t0 > timeout_s:
            return False
        time.sleep(0.01)
    return future.done()


def safe_shutdown():
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass


def to_duration(seconds):
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


class TrajectoryClient:
    """arm_controller FollowJointTrajectory action'i icin basit sarmalayici."""

    def __init__(self, node, joint_names, action_name=ACTION_NAME):
        self.node = node
        self.joint_names = list(joint_names)
        self.client = ActionClient(node, FollowJointTrajectory, action_name)
        self.action_name = action_name

    def wait_for_server(self, timeout_s=30.0):
        return self.client.wait_for_server(timeout_sec=timeout_s)

    def move(self, positions, duration_s, timeout_s=None):
        """Tek noktali yorunge gonderir, sonucu bekler. (basari, mesaj) dondurur."""
        return self.move_path([positions], [duration_s], timeout_s)

    def move_path(self, points, times, timeout_s=None):
        """Cok noktali yorunge: points[k] eklem konumlari, times[k] baslangictan sure [s].
        Ara noktalarda hiz verilmez (JTC spline ile gecer), son noktada hiz 0."""
        log = self.node.get_logger()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        for k, (positions, t) in enumerate(zip(points, times)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in positions]
            if k == len(points) - 1:
                pt.velocities = [0.0] * len(positions)
            pt.time_from_start = to_duration(t)
            goal.trajectory.points.append(pt)
        duration_s = float(times[-1])
        goal.goal_time_tolerance = to_duration(2.0)

        send_f = self.client.send_goal_async(goal)
        if not wait_future(send_f, 10.0):
            return False, 'goal gonderilemedi (zaman asimi)'
        handle = send_f.result()
        if handle is None or not handle.accepted:
            return False, 'goal reddedildi'
        res_f = handle.get_result_async()
        if timeout_s is None:
            timeout_s = duration_s + 15.0
        if not wait_future(res_f, timeout_s):
            log.warning('Sonuc beklenirken zaman asimi, goal iptal ediliyor')
            handle.cancel_goal_async()
            return False, 'sonuc zaman asimi'
        wrapped = res_f.result()
        res = wrapped.result
        ok = res.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        msg = 'OK' if ok else f'error_code={res.error_code} {res.error_string}'
        return ok, msg
