import rclpy
from rclpy.node import Node
from chapter4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge
import time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.brighe = CvBridge()
        self.test_image_path =os.path.join(get_package_share_directory('demo_python_service'), 
                                              'resource/test.jpg')
        self.get_logger().info(f"人脸检测客户端已启动")
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.test_image_path)

    def call_set_paramters(self, parameters):
        '''调用服务，修改参数'''
        # 首先创建一个客户端，等待服务上线
        update_param_client =  self.create_client(SetParameters, '/face_detect_node/set_parameters')
        while update_param_client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待参数更新服务上线')
        # 构造request函数
        request = SetParameters.Request()
        request.parameters = parameters
        # 发送请求并等待处理完成
        update_param_client

    def send_request(self):
        # 检测服务端是否上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待服务端上线')
        # 构造request函数
        request = FaceDetector.Request()
        request.image = self.brighe.cv2_to_imgmsg(self.image)
        # 发送请求并等待处理完成
        future = self.client.call_async(request=request)
        # rclpy.spin_until_future_complete(self,future=future)
        # response = future.result()
        # self.get_logger().info(f'接收到响应，共检测到{response.number}张人脸，耗时{response.use_time}s')
        # self.show_response(response)
        def result_callback(result_future):
            response = future.result()
            self.get_logger().info(f'接收到响应，共检测到{response.number}张人脸，耗时{response.use_time}s')
            self.show_response(response)
        future.add_done_callback(result_callback)


    def show_response(self, response):
        for i in range(response.number):
            top = response.top[i]
            left = response.left[i]
            right = response.right[i]
            bottom = response.bottom[i]
            cv2.rectangle(self.image,(left,top),(right,bottom),(255, 0), 4)
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0)

def main():
    rclpy.init()
    node = FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()