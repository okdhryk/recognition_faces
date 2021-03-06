#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
/image_raw ノードからカラー画像を読み込む

OpenCVで使うBGR系からface_recognitionで使うRGB系に変換する
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")




/image_gray ノードに白黒画像を出力する

roslaunch uvc_camera uvc_camera_node でテストできる
"""
import os
import glob
import rospy
import roslib.packages
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import face_recognition


#パッケージのディレクトリ
pkg_dir = roslib.packages.get_pkg_dir('recognition_faces')

#known_faces  ディレクトリにあるファイルの一覧
known_faces = glob.glob(pkg_dir+'/data/known_faces/*.jpg')
known_face_number =  len( known_faces )

known_face_names =[]
for f in glob.glob(pkg_dir+'/data/known_faces/*.jpg'):
    print(os.path.split(f)[1])  #ファイル名
    print (os.path.splitext(os.path.basename(f))[0]) #拡張子なし（氏名）
    known_face_names.append(os.path.splitext(os.path.basename(f))[0]) #拡張子なし（氏名）

known_face_images = []
for face in known_faces:
    known_face_images.append( face_recognition.load_image_file(face))
known_face_encodings = []    
for face_image in known_face_images:
    known_face_encodings.append( face_recognition.face_encodings(face_image)[0])
    


def recognition_face(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#        rgb_frame = cv_image[:, :, ::-1]
        rgb_frame = cv_image

        #画像の大きさ
        height, width = cv_image.shape[:2]
        print  height, width 

        # Find all the faces and face enqcodings in the frame of video
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        # Loop through each face in this frame of video
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            name = "Unknown"
    
            # If a match was found in known_face_encodings, just use the first one.
            # if True in matches:
            #     first_match_index = matches.index(True)
            #     name = known_face_names[first_match_index]

            # Or instead, use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]
                print (1.0 - face_distances[best_match_index] )*100 #信頼度　０〜１００
                print left, top #画面の左上が0,0 右下が


            # Draw a box around the face
            cv2.rectangle(cv_image, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(cv_image, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(cv_image, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)


                
        imgMsg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub = rospy.Publisher('image_gray', Image, queue_size=10)
        pub.publish(imgMsg)
    except Exception as err:
        print err

def start_node():
    rospy.init_node('recognition_faces')
    rospy.loginfo('recognition face node started')
    rospy.Subscriber("image_raw", Image, recognition_face)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
