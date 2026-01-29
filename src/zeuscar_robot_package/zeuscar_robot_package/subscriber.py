"""Subscriber node for Arduino serial communication."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time


class MinimalSubscriber(Node):
    """
    文字列メッセージを受信し、それをArduinoへシリアル送信するROS2サブスクライバノード.
    """

    def __init__(self):
        """ノードの初期化."""
        # ノード名 'minimal_subscriber' で初期化
        super().__init__('minimal_subscriber')

        # サブスクライバの作成
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

        # シリアルポート設定
        serial_port = '/dev/ttyACM0'  # Arduinoのデフォルトポート
        baud_rate = 9600

        try:
            self.ser = serial.Serial(serial_port, baud_rate)
            # Arduinoリセット直後などの待ち時間を考慮
            time.sleep(2)
            self.get_logger().info(
                f"Serial connection established on {serial_port} at {baud_rate} bps."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None  # ポートが開けなかった場合はNoneにしておく

    def listener_callback(self, msg):
        """
        Subscriberで受信した文字列をArduinoへシリアル送信.

        Args:
            msg: 受信したStringメッセージ
        """
        self.get_logger().info(f"I heard: '{msg.data}'")

        # シリアルポートが正常に開いている場合のみ送信
        if self.ser is not None and self.ser.is_open:
            try:
                # 文字列をそのまま送信
                self.ser.write(msg.data.encode())
                self.get_logger().info(f"Sent to Arduino: {msg.data}")

                # 必要があればArduinoの応答を受け取る
                time.sleep(0.1)
                if self.ser.in_waiting > 0:
                    received_data = self.ser.readline().decode(
                        'utf-8', errors='ignore'
                    ).rstrip()
                    self.get_logger().info(f"Received from Arduino: {received_data}")
            except Exception as e:
                self.get_logger().error(f"Serial write/read error: {e}")
        else:
            self.get_logger().warning(
                "Serial port is not open. Unable to send command."
            )


def main(args=None):
    """メイン関数."""
    # ROS2の初期化
    rclpy.init(args=args)

    # MinimalSubscriberノードをインスタンス化
    minimal_subscriber = MinimalSubscriber()

    # ノードをスピンさせ、コールバックを待機
    rclpy.spin(minimal_subscriber)

    # ノード終了処理
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
