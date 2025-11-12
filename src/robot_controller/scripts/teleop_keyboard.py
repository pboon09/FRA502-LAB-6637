#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ControlMode
from geometry_msgs.msg import Twist
import sys, select, termios, tty, threading, os
import numpy as np

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.mode_client = self.create_client(ControlMode, '/set_control_mode')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_panel = 0  # 0=IPK, 1=TO, 2=AM
        self.running = True
        self.speed = 0.05  # base speed for movement
        self.lock = threading.Lock()

        self.interactive_mode = sys.stdin.isatty()
        if self.interactive_mode:
            self.settings = termios.tcgetattr(sys.stdin)

        self.show_panel(self.current_panel)
        self.status("ready")

        # Initial position values for x, y, z (all starting at 0.0)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.effector_frame_mode = True  # Default to Effector Frame

        # Joint velocity limit (1 rad/s)
        self.vel_limit = 1.0  # in rad/s
        self.step_size = 0.05  # Step size for control

        # Set the fixed publishing rate to 100 Hz
        self.timer = self.create_timer(1.0 / 100.0, self.timer_callback)

    def clear(self):
        os.system('clear' if os.name == 'posix' else 'cls')

    def show_panel(self, panel):
        self.clear()
        if panel == 0:
            print(
                "==============================\n"
                "          IPK PANEL           \n"
                "==============================\n"
                "  1) IPK (this panel)\n"
                "  2) TO\n"
                "  3) AM\n"
                "------------------------------\n"
                "  q : quit\n"
                "  r : enter target coordinates for IPK\n"
                "==============================\n"
            )
        elif panel == 1:
            print(
                "==============================\n"
                "          TO PANEL            \n"
                "==============================\n"
                "  1) IPK\n"
                "  2) TO (this panel)\n"
                "  3) AM\n"
                "------------------------------\n"
                "  i/k : +x / -x\n"
                "  j/l : +y / -y\n"
                "  u/o : +z / -z\n"
                "  space: stop\n"
                "  w : switch to World Frame\n"
                "  e : switch to Effector Frame\n"
                "  q : quit\n"
                "==============================\n"
            )
        elif panel == 2:
            print(
                "==============================\n"
                "          AM PANEL            \n"
                "==============================\n"
                "  1) IPK\n"
                "  2) TO\n"
                "  3) AM (this panel)\n"
                "------------------------------\n"
                "  a : start/step auto (mode=2)\n"
                "  q : quit\n"
                "==============================\n"
            )

    def status(self, text):
        line = f"[panel={['IPK','TO','AM'][self.current_panel]}] {text}"
        print("\r" + line.ljust(80), end="", flush=True)

    def getKey(self):
        if not self.interactive_mode:
            return ''
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            key = sys.stdin.read(1) if rlist else ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key.lower()
        except Exception:
            return ''

    def process_key(self, key):
        if key == 'q':
            self.request_mode(9)
            self.running = False
            print("\nexit")
            return

        if key in ('1', '2', '3'):
            self.request_mode(9)
            self.current_panel = int(key) - 1
            self.show_panel(self.current_panel)
            self.status("panel switched")
            return

        if self.current_panel == 0:
            self.handle_ipk(key)
        elif self.current_panel == 1:
            self.handle_teleop(key)
        elif self.current_panel == 2:
            self.handle_auto_mode(key)

    def handle_ipk(self, key):
        if key == 'r':
            self.status("Enter x, y, z coordinates for IPK (press enter after each value)")
            self.x = float(input("Enter x: "))
            self.y = float(input("Enter y: "))
            self.z = float(input("Enter z: "))
            self.request_mode(0, self.x, self.y, self.z)
            self.status(f"IPK requested with x={self.x} y={self.y} z={self.z}")

    def handle_teleop(self, key):
        changed = False
        if key == 'i':
            self.x = min(self.x + self.step_size, 1.0)
            changed = True
        elif key == 'k':
            self.x = max(self.x - self.step_size, -1.0)
            changed = True
        elif key == 'j':
            self.y = min(self.y + self.step_size, 1.0)
            changed = True
        elif key == 'l':
            self.y = max(self.y - self.step_size, -1.0)
            changed = True
        elif key == 'u':
            self.z = min(self.z + self.step_size, 1.0)
            changed = True
        elif key == 'o':
            self.z = max(self.z - self.step_size, -1.0)
            changed = True
        elif key == ' ':
            self.x = self.y = self.z = 0.0
            changed = True
        elif key == 'w': 
            self.effector_frame_mode = False
            self.x = self.y = self.z = 0.0
            self.status("Switched to World Frame")
            self.request_mode(1)
            return
        elif key == 'e':
            self.effector_frame_mode = True
            self.x = self.y = self.z = 0.0
            self.status("Switched to Effector Frame")
            self.request_mode(2)
            return

        if changed:
            self.status(f"\rCurrent Mode: {'Effector Frame' if self.effector_frame_mode else 'World Frame'} Velocity updated: {self.x:.2f}, {self.y:.2f}, {self.z:.2f}")
            self.publish_velocity()

    def handle_auto_mode(self, key):
        if key == 'a':
            self.request_mode(2)
            self.status("AM requested")

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.linear.y = self.y
        twist.linear.z = self.z
        self.cmd_pub.publish(twist)

    def request_mode(self, mode, x=None, y=None, z=None):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.status("service unavailable")
            return
        req = ControlMode.Request()
        req.mode = mode
        if x is not None and y is not None and z is not None:
            req.x = x
            req.y = y
            req.z = z
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_response)

    def mode_response(self, future):
        try:
            res = future.result()
            if res.success and res.current_mode != 9:
                if res.current_mode == 0:
                    self.status(f"controller mode={res.current_mode} : {res.message} with solution {', '.join([f'{x:.2f}' for x in res.q_solution])}")
                else:
                    self.status(f"controller mode={res.current_mode} : {res.message}")
            else:
                self.status(f"controller error: {res.message}")
        except Exception as e:
            self.status(f"service error: {e}")

    def keyboard_loop(self):
        try:
            while self.running and rclpy.ok():
                key = self.getKey()
                if key:
                    with self.lock:
                        self.process_key(key)
        except KeyboardInterrupt:
            self.running = False
        finally:
            if self.interactive_mode:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nstopped")

    def timer_callback(self):
        self.publish_velocity()

    def run(self):
        thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        thread.start()
        while self.running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        thread.join(timeout=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
