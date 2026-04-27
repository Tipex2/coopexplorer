import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
import math

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        # Parámetros
        self.declare_parameter('robot_namespace', 'tb3_0')
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('frontier_threshold', 0.3)
        ns = self.get_parameter('robot_namespace').value

        # Suscriptor al mapa
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Publicador de marcadores para RViz2
        self.marker_pub = self.create_publisher(
            MarkerArray, '/frontiers', 10)

        # Action client de Nav2
        self.nav_client = ActionClient(
            self, NavigateToPose, f'/navigate_to_pose')

        self.map_data = None
        self.navigating = False
        self.get_logger().info(f'FrontierDetector iniciado para {ns}')

        # Timer que analiza el mapa cada 3 segundos
        self.timer = self.create_timer(5.0, self.explore)

    def map_callback(self, msg):
        self.map_data = msg

    def explore(self):
        if self.map_data is None or self.navigating:
            return

        frontiers = self.detect_frontiers()
        if not frontiers:
            self.get_logger().info('No se encontraron fronteras')
            return

        clusters = self.cluster_frontiers(frontiers)
        if not clusters:
            return

        self.publish_markers(clusters)

        # Elegir el cluster más grande
        best = max(clusters, key=lambda c: len(c))
        goal_x, goal_y = self.centroid(best)

        if not self.is_within_map(goal_x, goal_y):
            self.get_logger().warn(
                f'Frontera ({goal_x:.2f}, {goal_y:.2f}) fuera del mapa, ignorando')
            return

        self.get_logger().info(
            f'Navegando a frontera: ({goal_x:.2f}, {goal_y:.2f})')
        self.send_goal(goal_x, goal_y)

    def detect_frontiers(self):
        msg = self.map_data
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        frontiers = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = y * width + x
                # Celda libre
                if data[idx] != 0:
                    continue
                # Tiene al menos un vecino desconocido
                neighbors = [
                    data[(y-1)*width + x],
                    data[(y+1)*width + x],
                    data[y*width + (x-1)],
                    data[y*width + (x+1)],
                ]
                if -1 in neighbors:
                    wx = ox + (x + 0.5) * res
                    wy = oy + (y + 0.5) * res
                    frontiers.append((wx, wy))

        return frontiers

    def cluster_frontiers(self, frontiers, radius=0.5):
        clusters = []
        visited = set()

        for i, f in enumerate(frontiers):
            if i in visited:
                continue
            cluster = [f]
            visited.add(i)
            for j, g in enumerate(frontiers):
                if j in visited:
                    continue
                if math.dist(f, g) < radius:
                    cluster.append(g)
                    visited.add(j)
            min_size = self.get_parameter('min_frontier_size').value
            if len(cluster) >= min_size:
                clusters.append(cluster)

        return clusters

    def centroid(self, cluster):
        x = sum(p[0] for p in cluster) / len(cluster)
        y = sum(p[1] for p in cluster) / len(cluster)
        return x, y

    def is_within_map(self, x, y):
        msg = self.map_data
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        width = msg.info.width * msg.info.resolution
        height = msg.info.height * msg.info.resolution
        return (ox < x < ox + width) and (oy < y < oy + height)

    def publish_markers(self, clusters):
        arr = MarkerArray()
        for i, cluster in enumerate(clusters):
            cx, cy = self.centroid(cluster)
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def send_goal(self, x, y):

        self.get_logger().info('Esperando Nav2...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 disponible')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.navigating = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rechazado')
            self.navigating = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info('Goal completado, buscando nueva frontera')
        self.navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
