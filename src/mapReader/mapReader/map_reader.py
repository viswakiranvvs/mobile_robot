import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
import numpy as np
import matplotlib.pyplot as plt
import cv2
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from rtabmap_msgs.srv import GetMap as GetMapDataSrv  # Not nav_msgs GetMap
from nav_msgs.srv import GetMap as GetOccupancyMapSrv
from mpl_toolkits.mplot3d import Axes3D

class MapClient(Node):
    def __init__(self):
        super().__init__('map_client')
        
        # Create client for occupancy map
        self.occ_map_client = self.create_client(GetOccupancyMapSrv, '/rtabmap/rtabmap/get_map')
        self.cli = self.occ_map_client
        while not self.occ_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Occupancy Map service...')

        # Create client for RTAB-Map map data
        self.map_data_client = self.create_client(GetMapDataSrv, '/rtabmap/rtabmap/get_map_data')
        while not self.map_data_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Map Data service...')

        # self.get_logger().info('Calling both services...')

    def send_request(self):
        self.req = GetMap.Request()
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            map_msg = future.result().map
            self.get_logger().info('Map received:')
            # Access all fields from the message
            print("Header Frame ID:", map_msg.header.frame_id)
            print("Map Resolution:", map_msg.info.resolution)
            print("Map Width x Height:", map_msg.info.width, 'x', map_msg.info.height)
            print("Origin Position:", map_msg.info.origin.position.x,
                  map_msg.info.origin.position.y, map_msg.info.origin.position.z)
            print("Origin Orientation:", map_msg.info.origin.orientation)
            print("Data length:", len(map_msg.data))

            # Save as Python structured object
            self.map_data = {
                "frame_id": map_msg.header.frame_id,
                "resolution": map_msg.info.resolution,
                "width": map_msg.info.width,
                "height": map_msg.info.height,
                "origin": {
                    "position": {
                        "x": map_msg.info.origin.position.x,
                        "y": map_msg.info.origin.position.y,
                        "z": map_msg.info.origin.position.z
                    },
                    "orientation": {
                        "x": map_msg.info.origin.orientation.x,
                        "y": map_msg.info.origin.orientation.y,
                        "z": map_msg.info.origin.orientation.z,
                        "w": map_msg.info.origin.orientation.w
                    }
                },
                "data": list(map_msg.data)  # convert to list of int8
            }
            self.visualize_occupancy_grid(self.map_data)

        else:
            self.get_logger().error('Service call failed')
    
    def visualize_occupancy_grid(self,map_data):
        width = map_data["width"]
        height = map_data["height"]
        data = map_data["data"]

        # Convert to numpy array and reshape to (height, width)
        grid = np.array(data, dtype=np.int8).reshape((height, width))

        # Convert to displayable image (0=free → white, 100=occupied → black, -1=unknown → gray)
        img = np.zeros((height, width), dtype=np.uint8)
        img[grid == -1] = 128  # Unknown = gray
        img[grid == 0] = 255   # Free = white
        img[grid > 0] = 0      # Occupied = black

        # Flip vertically (to match RViz view)
        img = np.flipud(img)

        # Show using OpenCV or matplotlib
        # cv2.imshow("Occupancy Grid", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # OR matplotlib (if headless)
        plt.imshow(img, cmap='gray')
        plt.title("RTAB-Map Occupancy Grid")
        plt.show()
    
    def call_get_occupancy_map(self):
        request = GetOccupancyMapSrv.Request()
        future = self.occ_map_client.call_async(request)
        print("Calling occupancy service")
        rclpy.spin_until_future_complete(self, future)
        self.handle_occupancy_response(future)
        # future.add_done_callback(self.handle_occupancy_response)

    def call_get_map_data(self):
        request = GetMapDataSrv.Request()
        request.global_map = True
        request.optimized = True
        request.graph_only = False
        self.get_logger().info("Calling Map Data Service")
        future = self.map_data_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Received, calling handler")
        self.handle_map_data_response(future)
        
        # future.add_done_callback(self.handle_map_data_response)

    def handle_occupancy_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received Occupancy Grid map: {response.map.info.width}x{response.map.info.height}")
            # print(f"Received Occupancy Grid map: {response.map.info.width}x{response.map.info.height}")
            self.map_data = {
                "width": response.map.info.width,
                "height": response.map.info.height,
                "resolution": response.map.info.resolution,
                "origin": {
                    "x": response.map.info.origin.position.x,
                    "y": response.map.info.origin.position.y
                },
                "data": response.map.data
            }
            # You can visualize or save it here
            self.visualize_occupancy_grid(self.map_data)
        except Exception as e:
            print(f"Failed to get occupancy map: {e}")
            self.get_logger().error(f"Failed to get occupancy map: {e}")

    def visualize_map(self,data):
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, projection='3d')

        poses_x, poses_y, poses_z = [], [], []
        pts_x, pts_y, pts_z = [], [], []

        for node in data.nodes:
            # Extract pose position (robot location)
            pos = node.pose.position
            poses_x.append(pos.x)
            poses_y.append(pos.y)
            poses_z.append(pos.z)

            # Extract 3D feature points
            for pt in node.word_pts:
                pts_x.append(pt.x)
                pts_y.append(pt.y)
                pts_z.append(pt.z)

        # Plot robot poses
        ax.scatter(poses_x, poses_y, poses_z, c='blue', marker='o', label='Robot Poses')

        # Plot 3D feature points
        ax.scatter(pts_x, pts_y, pts_z, c='red', s=1, label='3D Map Points')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('RTAB-Map: Node Poses & 3D Landmarks')
        ax.legend()
        plt.tight_layout()
        plt.show()

    def plot_map_graph(self,graph):
        id_to_pose = {}  # Map from ID to (x, y) for fast lookup

        # Extract poses
        for i, node_id in enumerate(graph.poses_id):
            pos = graph.poses[i].position
            id_to_pose[node_id] = (pos.x, pos.y)

        fig, ax = plt.subplots(figsize=(10, 8))

        # Plot poses
        xs = [pos[0] for pos in id_to_pose.values()]
        ys = [pos[1] for pos in id_to_pose.values()]
        ax.scatter(xs, ys, c='blue', label='Optimized Poses')

        # Draw links (edges)
        for link in graph.links:
            from_id = link.from_id
            to_id = link.to_id

            if from_id in id_to_pose and to_id in id_to_pose:
                x0, y0 = id_to_pose[from_id]
                x1, y1 = id_to_pose[to_id]
                ax.plot([x0, x1], [y0, y1], c='gray', linewidth=0.8)

        ax.set_title('Optimized Pose Graph from RTAB-Map')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.axis('equal')
        ax.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    def handle_map_data_response(self, future):
        try:
            response = future.result()
            data = response.data
            self.get_logger().info(f"Received MapData: {len(data.nodes)} nodes, {len(data.graph.poses)} graph poses")

            # You can access the graph and node info like:
            # print(data.nodes[0])
            for i, node in enumerate(data.nodes[:5]):
                self.get_logger().info(f"Node {node.id}: pose = {node.pose}") # {node.pose.pose.position.y}

            # for i in range(0,5):
            #     self.get_logger().info(f"Pose Id {data.graph.poses_id[i]}")
            #     self.get_logger().info(f"Pose {data.graph.poses[i]}")
                # self.get_logger().info(f"Pose Id {data.graph.poses[0].id}: pose = {pose}") # {node.pose.pose.position.y}
            # self.visualize_map(data)
            self.plot_map_graph(data.graph)

        except Exception as e:
            self.get_logger().error(f"Failed to get map data: {e}")

def main(args=None):
    rclpy.init(args=args)
    map_client = MapClient()
    # map_client.send_request()
    # map_client.call_get_occupancy_map()
    map_client.call_get_map_data()
    map_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
