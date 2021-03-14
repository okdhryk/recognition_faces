#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
登録してある顔画像ファイルとマッチングして人物認証を行う

/image_raw ノードからカラー画像を読み込む
/image_face ノードに画像を出力する




OpenCVで使うBGR系からface_recognitionで使うRGB系に変換する
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

roslaunch uvc_camera uvc_camera_node でテストできる
"""
import os
import glob
import rospy
import roslib.packages
import rosparam
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import face_recognition
from recognition_faces.msg import *
from std_msgs.msg import String
from std_msgs.msg import *




#顔認証用のカスタマイズメッセージのテスト
#face=Face()
#face.name="Unknown"


#fs=Faces()
#fs.faces=[]
#fs.faces.append(face)
#fs.faces.append(face)

#print fs





# 認識結果を画像としてトピックにPublishするか．デバッグ用
# 画像をPublishすると処理が遅くなる
DO_DISPLAY=rosparam.get_param("/recog_faces/do_display")
print DO_DISPLAY

#顔認証のしきい値（0〜１）
#マッチ距離がこれ以下だとUnknownになる
#ベストマッチアルゴリズム（登録してある画像の中で一番のマッチ度の人物と認証する）なので 0.5でもOKか？
RECOGNITION_RELIABILITY=rosparam.get_param("/recog_faces/recognition_reliability" )


#パッケージのディレクトリ
pkg_dir = roslib.packages.get_pkg_dir('recognition_faces')

#known_faces  既知の顔を登録するためのディレクトリにあるファイルの一覧
#［氏名］.jpgのような形式
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
    


#/image_raw ノードにカラー画像がPublishされるたびに一回だけ実行される
def recognition_face(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#        rgb_frame = cv_image[:, :, ::-1]

        #画像の大きさ
        height, width = cv_image.shape[:2]

        # 画像中の顔をすべて検出する
        face_locations = face_recognition.face_locations(cv_image, model="cnn")
        #face_locations = face_recognition.face_locations(cv_image, model="hog")
        face_encodings = face_recognition.face_encodings(cv_image, face_locations)

 
        faces=Faces()
        
        # 検出したすべての顔領域に関して，登録してある顔画像とマッチングする
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            name = "Unknown"
    
            # 最初にマッチした顔画像を認証とする場合
            # if True in matches:
            #     first_match_index = matches.index(True)
            #     name = known_face_names[first_match_index]

            # 入力画像とマッチ画像の距離が一番小さい人を認証とする場合
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                print (1.0 - face_distances[best_match_index] )*100 #信頼度　０〜１００
                
                if  face_distances[best_match_index] < (1.0 - RECOGNITION_RELIABILITY):
                    name = known_face_names[best_match_index]
                    face=Face()
                    face.name=name
                    face.title="professor"
                    face.reliability = 1.0 - face_distances[best_match_index]
                    face.gender="male"
                    face.age=99
                    face.posLeftTop.x=left
                    face.posLeftTop.y=top
                    face.posRightBottom.x=right
                    face.posRightBottom.y=bottom
                    face.height=height
                    face.width=width
                    faces.faces.append(face)
            
            if DO_DISPLAY:
                # 顔の領域に四角を書く
                cv2.rectangle(cv_image, (left, top), (right, bottom), (0, 0, 255), 2)

                # 顔の下に名前を書く（マッチしない場合はUnknown）
                cv2.rectangle(cv_image, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(cv_image, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
        
        #デバッグ用
        #print "found ", len(faces.faces), "people"
        #print faces
        
        if len(faces.faces) != 0:
            pub1 = rospy.Publisher('reco_face1',Faces , queue_size=1)
            pub1.publish(faces)
            
            if DO_DISPLAY:
                imgMsg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
                pub = rospy.Publisher('image_face', Image, queue_size=1)
                pub.publish(imgMsg)
                    
    except Exception as err:
        print err

def start_node():
    rospy.init_node('recog_faces')
    rospy.loginfo('<<<--- recognition face node started')
    rospy.Subscriber("image_raw", Image, recognition_face)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
