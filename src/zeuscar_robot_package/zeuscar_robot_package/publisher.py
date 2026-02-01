"""
ZeusCar Publisher Node

キーボードから入力されたコマンド番号（0-10）を
ROS2トピックにパブリッシュするノード。

コマンド一覧:
    0: FORWARD      - 前進
    1: BACKWARD     - 後退
    2: LEFT         - 左旋回
    3: RIGHT        - 右旋回
    4: LEFTFORWARD  - 左前進
    5: RIGHTFORWARD - 右前進
    6: LEFTBACKWARD - 左後退
    7: RIGHTBACKWARD- 右後退
    8: TURNLEFT     - 左回転
    9: TURNRIGHT    - 右回転
    10: STOP        - 停止
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# コマンド番号とArduinoコマンド文字列のマッピング
COMMANDS_MAP = {
    0:  "FORWARD",
    1:  "BACKWARD",
    2:  "LEFT",
    3:  "RIGHT",
    4:  "LEFTFORWARD",
    5:  "RIGHTFORWARD",
    6:  "LEFTBACKWARD",
    7:  "RIGHTBACKWARD",
    8:  "TURNLEFT",
    9:  "TURNRIGHT",
    10: "STOP"
}


class MinimalPublisher(Node):
    """
    文字列メッセージを送信するROS2パブリッシャノード
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        # パブリッシャーの作成
        # トピック名: 'topic'
        # メッセージ型: String
        # キューサイズ: 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        self.get_logger().info('MinimalPublisher node has been started.')

    def publish_command(self, command_str: str):
        """
        Arduino側で定義された文字列コマンドをROS2トピックへPublish

        Args:
            command_str: Arduinoコマンド文字列（例: "FORWARD", "STOP"）
        """
        msg = String()
        msg.data = command_str
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: "{command_str}"')


def main(args=None):
    """
    メイン関数: ノードを起動してキーボード入力を受け付ける
    """
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        while rclpy.ok():
            # ROS2の内部処理を実行（ノンブロッキング）
            rclpy.spin_once(minimal_publisher, timeout_sec=0.1)

            # ユーザーからのコマンド入力を受け付け
            user_input = input(
                "Enter command number (0=FORWARD, 1=BACKWARD, etc.) or 'exit': "
            ).strip()

            # 終了コマンド
            if user_input.lower() == 'exit':
                print("Exiting command input loop.")
                break

            # 入力値の検証とパブリッシュ
            if user_input.isdigit():
                cmd_num = int(user_input)
                if cmd_num in COMMANDS_MAP:
                    command_str = COMMANDS_MAP[cmd_num]
                    minimal_publisher.publish_command(command_str)
                else:
                    print("Invalid number. Please enter a value between 0 and 10.")
            else:
                print("Invalid input. Please enter a number (0-10) or 'exit'.")

    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
