import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os

def main():
    # 获取图片真实路径
    default_image_path =os.path.join(get_package_share_directory('demo_python_service'), 'resource/default.jpg')
    
    # 使用opencv加载图像
    image = cv2.imread(default_image_path)
    # 查找图像中的所有人脸 
    face_location = face_recognition.face_locations(image,number_of_times_to_upsample=1,model='hog')
    # 绘制每张人脸的边框
    for top, right, bottom, left in face_location:
        cv2.rectangle(image,(left,top),(right,bottom),(255, 0), 4)
    # 显示图像
    cv2.imshow('Face Detection', image)
    cv2.waitKey(0)
