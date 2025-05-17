import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, ColorRGBA
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory

from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from tensorflow.compat.v1 import InteractiveSession, ConfigProto
import tensorflow as tf

import cv2
import numpy as np
import threading
import time
import zipfile
import json

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.image_size = 24
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        # GPU konfiguráció
        config = ConfigProto()
        config.gpu_options.allow_growth = True
        self.session = InteractiveSession(config=config)

        # Modell betöltése
        pkg_path = get_package_share_directory('tb3_project_py')
        model_path = pkg_path + "/network_model/model.best.keras"

        keras_version_str = tf.keras.__version__
        print("TensorFlow:", tf.__version__)
        print("Keras:", keras_version_str)
        print("Model path:", model_path)

        model_version = self.get_keras_version_from_keras_file(model_path)
        if model_version != keras_version_str:
            print(f"Model was built with Keras {model_version}, but you're using {keras_version_str}.")
            exit()

        self.model = load_model(model_path, compile=False)
        self.activation_model_1 = tf.keras.models.Model(
            inputs=self.model.inputs, outputs=self.model.get_layer("conv2d_1").output)
        self.activation_model_2 = tf.keras.models.Model(
            inputs=self.model.inputs, outputs=self.model.get_layer("activation_1").output)

        self.model.summary()
        self.last_time = time.time()

        # ROS
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            1
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.color_publisher = self.create_publisher(Int32, 'line_color', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def get_keras_version_from_keras_file(self, path):
        with zipfile.ZipFile(path, 'r') as archive:
            if 'metadata.json' in archive.namelist():
                with archive.open('metadata.json') as f:
                    metadata = json.load(f)
                    return metadata.get('keras_version', 'Unknown')
            return 'Unknown'

    def image_callback(self, msg):
        with self.frame_lock:
            self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def spin_thread_func(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def display_image(self):
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800, 600)

        while rclpy.ok():
            if self.latest_frame is not None:
                grid_1, grid_2 = self.process_image(self.latest_frame)
                result = self.add_small_pictures(self.latest_frame, [grid_1, grid_2], size=(260, 125))
                cv2.imshow("frame", result)
                self.latest_frame = None

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
                break

        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, img):
        msg = Twist()
        image = cv2.resize(img, (self.image_size, self.image_size))
        image = img_to_array(image) / 255.0
        image = image.reshape(-1, self.image_size, self.image_size, 3)

        preds = self.model.predict(image, verbose=0)[0]
        prediction_direction = int(np.argmax(preds[0:4]))
        prediction_color = int(np.argmax(preds[4:7]))

        # Aktivációs rétegek megjelenítéséhez
        activations_1 = self.activation_model_1.predict(image)
        activations_2 = self.activation_model_2.predict(image)
        grid_image_1 = self.visualize_feature_maps(activations_1, num_rows=5, num_cols=10, padding=1)
        grid_image_2 = self.visualize_feature_maps(activations_2, num_rows=5, num_cols=10, padding=1)

        # Irányvezérlés – sebesség fix
        if prediction_direction == 0:  # Forward
            msg.linear.x = 0.08
            msg.angular.z = 0.0
        elif prediction_direction == 1:  # Left
            msg.linear.x = 0.05
            msg.angular.z = -0.3
        elif prediction_direction == 2:  # Right
            msg.linear.x = 0.05
            msg.angular.z = 0.3
        else:  # Nothing
            msg.linear.x = 0.0
            msg.angular.z = 0.2

        self.publisher.publish(msg)

        # Szín Int32 topicon
        color_msg = Int32()
        color_msg.data = prediction_color
        self.color_publisher.publish(color_msg)

        # Szín RViz markerrel
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "line_color_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color = self.get_color_rgba(prediction_color)
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)

        color_names = ["Red", "Yellow", "Blue"]
        print(f"Predicted direction: {prediction_direction}, color: {color_names[prediction_color]}")

        self.last_time = time.time()
        return grid_image_1, grid_image_2

    def get_color_rgba(self, color_index):
        color_map = {
            0: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Red
            1: ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),  # Yellow
            2: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Blue
        }
        return color_map.get(color_index, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))  # Default: White

    def visualize_feature_maps(self, activations, num_rows=2, num_cols=10, padding=2):
        activations = activations[0]
        feature_map_size = activations.shape[1]
        grid_image = np.zeros(((feature_map_size + padding) * num_rows - padding,
                               (feature_map_size + padding) * num_cols - padding, 3), dtype=np.uint8)

        for i in range(activations.shape[-1]):
            feature_map = activations[:, :, i]
            feature_map = (feature_map - feature_map.min()) / (feature_map.max() - feature_map.min())
            heatmap = cv2.applyColorMap((feature_map * 255).astype(np.uint8), cv2.COLORMAP_TURBO)

            row = i // num_cols
            col = i % num_cols
            x_start = col * (feature_map_size + padding)
            y_start = row * (feature_map_size + padding)
            grid_image[y_start:y_start+feature_map_size, x_start:x_start+feature_map_size, :] = heatmap

        return grid_image

    def add_small_pictures(self, img, small_images, size=(160, 120)):
        x_base_offset, y_base_offset = 40, 10
        x_offset = x_base_offset
        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))
            img[y_base_offset:y_base_offset+size[1], x_offset:x_offset+size[0]] = small
            x_offset += size[0] + x_base_offset
        return img

    def stop_robot(self):
        self.publisher.publish(Twist())

    def stop(self):
        self.running = False
        self.stop_robot()
        self.spin_thread.join()

def main(args=None):
    print("OpenCV version:", cv2.__version__)
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        node.display_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
