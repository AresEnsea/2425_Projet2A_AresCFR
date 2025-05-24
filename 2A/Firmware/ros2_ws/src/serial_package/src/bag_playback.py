#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import serial
import os

class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')

        # ─── State ─────────────────────────────────────────────────────────────
        self.bag_path = None
        self.tirette_value = None
        self.bag_started = False
        self.bag_process = None
        self.motor_stopped = False

        # Serial port configuration (opened per-send)
        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

        # ─── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            String, '/file_stm_choisi', self.file_stm_callback, 10
        )
        self.create_subscription(
            Bool,   '/tirette_value',   self.tirette_callback, 10
        )
        self.create_subscription(
            String, '/keyboard_commands', self.command_callback, 10
        )
        self.create_subscription(
            Bool,   '/stop_moteur',      self.stop_moteur_callback, 10
        )

        # ─── Compute “bags/” folder relative to this script ───────────────
        this_file = os.path.realpath(__file__)
        src_dir   = os.path.dirname(this_file)       # …/serial_package/src
        pkg_root  = os.path.dirname(src_dir)         # …/serial_package
        self.bags_dir = os.path.join(pkg_root, 'bags')  # …/serial_package/bags
        self.get_logger().info(f"Expecting bags under: {self.bags_dir}")

        # ─── Heartbeat timer (1 Hz) ─────────────────────────────────────────
        self.create_timer(1.0, self._heartbeat)

        # ─── Stop‐packet timer (10 Hz) ───────────────────────────────────────
        #    Keeps sending a 26‐zero packet whenever motor_stopped == True
        self.create_timer(0.1, self._stop_timer)

        self.get_logger().info("CombinedNode started: waiting for /file_stm_choisi and /tirette_value…")

    def _heartbeat(self):
        # Prints once per second so you know the node is alive
        self.get_logger().info("Node is alive (waiting for topics…)")

    def _stop_timer(self):
        # This runs at 10 Hz. If motor_stopped is True, send a 26‐zero packet immediately.
        if self.motor_stopped:
            packet = "0" * 26
            full = f">{packet}"
            try:
                with serial.Serial(self.uart2_port, self.baudrate, timeout=self.timeout) as ser:
                    ser.write(full.encode())
                    # (No need to log every 0.1 s; maybe debug‐level if desired)
                    self.get_logger().debug(f"Stop‐timer ⟶ {full}")
            except serial.SerialException as e:
                self.get_logger().error(f"Error opening serial port (stop timer): {e}")

    def file_stm_callback(self, msg: String):
        bag_filename = msg.data.strip()
        if not bag_filename:
            self.get_logger().warn("Received empty /file_stm_choisi. Still waiting for a valid filename.")
            return

        full_path = os.path.join(self.bags_dir, bag_filename)
        if not os.path.exists(full_path):
            self.get_logger().warn(f"Bag not found at: {full_path}")
            return

        self.bag_path = full_path
        self.get_logger().info(f"Using bag: {self.bag_path}")
        self._maybe_start_bag()

    def tirette_callback(self, msg: Bool):
        self.tirette_value = msg.data
        self.get_logger().info(f"Received /tirette_value → {self.tirette_value}")
        self._maybe_start_bag()

    def _maybe_start_bag(self):
        if self.bag_started:
            return

        if not self.bag_path:
            self.get_logger().info("Still waiting for a valid /file_stm_choisi…")
            return

        if not os.path.exists(self.bag_path):
            self.get_logger().warn(f"Bag path does not exist: {self.bag_path}")
            return

        if self.tirette_value is None:
            self.get_logger().info("Still waiting for /tirette_value…")
            return

        if self.tirette_value is False:
            self.get_logger().info(f"Conditions met → playing bag: {self.bag_path}")
            try:
                self.bag_process = subprocess.Popen(
                    ['ros2', 'bag', 'play', self.bag_path],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE
                )
                self.bag_started = True
                self.get_logger().info("Bag playback started.")
            except Exception as e:
                self.get_logger().error(f"Failed to start bag playback: {e}")
        else:
            self.get_logger().info("Tirette is still True → waiting for it to become False")

    def command_callback(self, msg: String):
        """
        Whenever a /keyboard_commands msg arrives and motor_stopped is False,
        send that 26-char (or padded) command once. If motor_stopped == True,
        the zero‐packet is already being streamed by _stop_timer().
        """
        if self.motor_stopped:
            # If motor_stopped is true, do nothing here—_stop_timer already sends zeros.
            self.get_logger().debug("command_callback: motor_stopped is true → skipping")
            return

        raw = msg.data.strip()
        if len(raw) >= 26:
            packet = raw[:26]
        else:
            packet = raw.ljust(26, "0")

        self.get_logger().info(f"Sending packet: {packet}")
        full = f">{packet}"
        try:
            with serial.Serial(self.uart2_port, self.baudrate, timeout=self.timeout) as ser:
                self.get_logger().info(f"Serial ⟶ {full}")
                ser.write(full.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")

    def stop_moteur_callback(self, msg: Bool):
        self.motor_stopped = msg.data
        self.get_logger().info(f"Received /stop_moteur → {self.motor_stopped}")
        # No need to send an immediate zero here; the 10 Hz _stop_timer will begin streaming
        # zeros on the next cycle (within 0.1 s).

    def destroy(self):
        # Terminate the bag process if still running
        if self.bag_process and (self.bag_process.poll() is None):
            self.get_logger().info("Terminating bag playback…")
            self.bag_process.terminate()
            self.bag_process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CombinedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt → shutting down")
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
