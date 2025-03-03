import rclpy
from rclpy.node import Node
from chapter4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge
import time
from rcl_interfaces.msg import SetParametersResult

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.service_=self.create_service(FaceDetector, 'face_detect', self.detect_face_callback)
        # self.number_of_times_to_upsample = 1
        # self.model = 'hog'
        self.bridge = CvBridge()
        self.default_image_path =os.path.join(get_package_share_directory('demo_python_service'), 
                                              'resource/default.jpg')
        self.get_logger().info("人脸检测服务已经启动")
        # 声明和获取参数
        self.declare_parameter('face_locations_upsample_times', 1)
        self.declare_parameter('face_locations_model',"hog")
        self.model=self.get_parameter("face_locations_model").value
        self.upsamle_times = self.get_parameter("face_locations_upsample_times").value
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(
                f'参数{parameter.name} 设置为：{parameter.value}'
            )
            if parameter.name == 'face_locations_upsample_times':
                self.upsamle_times = parameter.value
            if parameter.name == 'face_locations_model':
                self.model = parameter.value
        return SetParametersResult(successful=True)


    
    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info(f"传入图像为空，使用默认图像加载")
        
        start_time = time.time()
        self.get_logger().info(f"加载图像完成， 开始识别！")
        face_location = face_recognition.face_locations(cv_image,self.number_of_times_to_upsample,self.model)
        response.use_time = time.time() - start_time
        response.number = len(face_location)
        for top, right, bottom, left in face_location:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response


def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()