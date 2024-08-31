import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

from custom_message.msg import Gps
from custom_message.msg import Accelerometer

gps_x = [0]
gps_altitude = [0]
accelerometer_x = [0]
accelerometer_y = [0]

plt.ion()
fig, ax = plt.subplots()
ax2 = ax.twinx()
gps_line, = ax.plot([], [], "r")
accelerometer_line, = ax2.plot([], [], "b")

class GraphNode(Node):
    def __init__(self):
        super().__init__('graph_node')
        self.create_subscription(
            Gps,
            'gps_topic',
            self.gps_callback,
            10)
        self.create_subscription(
            Accelerometer,
            'accelerometer_topic',
            self.accelerometer_callback,
            10)


    def gps_callback(self, msg):
        gps_x.append(msg.time)
        gps_altitude.append(msg.altitude)
        while(gps_x[1] < msg.time - 20000):
            gps_x.pop(0)
            gps_altitude.pop(0)

        gps_line.set_xdata(gps_x)
        gps_line.set_ydata(gps_altitude)
        ax.set_xlim(max(gps_x) - 20000, max(gps_x))
        ax.set_ylim(min(gps_altitude)-1, max(gps_altitude)+1)
        fig.canvas.draw()
        fig.canvas.flush_events()

        self.get_logger().info(f"x= {msg.time}, y= {msg.altitude}")

    def accelerometer_callback(self, msg):
        accelerometer_x.append(msg.time)
        accelerometer_y.append(msg.y)
        while(accelerometer_x[1] < msg.time - 20000):
            accelerometer_x.pop(0)
            accelerometer_y.pop(0)

        accelerometer_line.set_xdata(accelerometer_x)
        accelerometer_line.set_ydata(accelerometer_y)
        ax2.set_xlim(max(accelerometer_x) - 20000, max(accelerometer_x))
        ax2.set_ylim(min(accelerometer_y)-1, max(accelerometer_y)+1)
        fig.canvas.draw()
        fig.canvas.flush_events()

def main(args=None):
    ax.set_xlabel("t[msec]")
    ax.set_ylabel("altitude[m]", color="red")
    ax2.set_ylabel("accelerometer[m/s^2]", color="blue")
    rclpy.init(args=args)

    minimal_subscriber = GraphNode()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()