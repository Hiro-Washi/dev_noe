#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import cv2
import collections
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
#from ros_openpose.msg import Frame
from cv_bridge import CvBridge, CvBridgeError
from happymimi_msgs.srv import SetStr, SetStrResponse

class DetectClothColor(object):
    def __init__(self):
        rospy.Service('/person_feature/cloth_color', SetStr, self.main)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        rospy.Subscriber('/frame', Frame, self.openPoseCB)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)

        self.image_res = Image()
        self.pose_res = Frame()

    def realsenseCB(self, res):
        self.image_res = res

    def openPoseCB(self, res):
        self.pose_res = res

    def judgeColor(self, req):
        # hsv色空間で色の判定
        h, s, v = req
        #print h, s, v
        color = ''
        if 0<=v and v<=79: color = 'Black'
        elif (0<=s and s<=50) and (190<=v and v<=255): color = 'White'
        elif (0<=s and s<=50) and (80<=v and v<=130): color = 'Gray'
        #elif (50 <= s and s <= 170) and (70 <= v and v <= 150): color = 'Gray'
        elif (50<=s and s<=170) and (80<=v and v<=90): color = 'Gray'
        #elif (0<=s and s<=50) and (80<=v and v<=230): color = 'Gray'
        #elif (5<=h and h<=18) and (20<=s and s<=240) and (70<=v and v<=180): color = 'Brown'
        elif (5<=h and h<=18) and v<=200: color = 'Brown'
        elif (0<=h and h<=4) or (174<=h and h<=180): color = 'Red'
        elif 5<=h and h<=18: color = 'Orange'
        elif 19<=h and h<=39: color = 'Yellow'
        elif 40<=h and h<=89: color = 'Green'
        elif 90<=h and h<=136: color = 'Blue'
        elif 137<=h and h<=159: color = 'Purple'
        elif 160<=h and h<=173: color = 'Pink'
        return color

    def main(self, _):
        response = SetStrResponse()

        self.head_pub.publish(-20.0)
        rospy.sleep(2.5)

        
        
        # neckとhipの座標から中点を得る →　色の認識のみなのでいらない
  


        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_map = ['']
        for i in range(chest_length+1):
            x = body_axis_x + i
            if x<0 or x>479: continue
            for j in range(-width, width):
                y = body_axis_y + j
                if y<0 or y>639: continue
                color = self.judgeColor(hsv_image[int(x), int(y)])
                color_map.append(color)
        print color_map
        count_l = collections.Counter(color_map)
        response.result = count_l.most_common()[0][0]

        return response

class RecognitionTools(object):
    bbox = []

    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.boundingBoxCB)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        rospy.Service('/recognition/save',StrTrg,self.saveImage)
        rospy.Service('/recognition/list',RecognitionList,self.listObject)
        rospy.Service('/recognition/find',RecognitionFind,self.findObject)
        rospy.Service('/recognition/count',RecognitionCount,self.countObject)
        rospy.Service('/recognition/localize',RecognitionLocalize,self.localizeObject)
        rospy.Service('/recognition/multiple_localize',MultipleLocalize,self.multipleLocalize)

        self.realsense_image = Image()
        self.image_height = 480# rosparam.get_param('/camera/realsense2_camera/color_height')
        self.image_width = 640# rosparam.get_param('/camera/realsense2_camera/color_width')
        try:
            self.object_dict = rosparam.get_param('/object_dict')
        except rosgraph.masterapi.MasterError:
            self.object_dict = {'any':['cup', 'bottle']}

        self.update_time = 0 # darknetからpublishされた時刻を記録
        self.update_flg = False # darknetからpublishされたかどうかの確認

        rospy.Timer(rospy.Duration(0.5), self.initializeBbox)

    def boundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        RecognitionTools.bbox = bb.bounding_boxes

    def initializeBbox(self, event):
        # darknetが何も認識していない時にRecognitionTools.bboxを初期化する
        if time.time() - self.update_time > 1.0 and self.update_flg:
            RecognitionTools.bbox = []
            self.update_flg = False
            rospy.loginfo('initialize')

    def createBboxList(self,bb):
        bbox_list = []
        for i in range(len(bb)):
            bbox_list.append(bb[i].Class)
        return bbox_list

    def realsenseCB(self, image):
        self.realsense_image = image

    def saveImage(self, req, bb=None):
        if bb is None:
            bb = RecognitionTools.bbox
        bbox_list = self.createBboxList(bb)

        bridge = CvBridge()
        cv2_image = bridge.imgmsg_to_cv2(self.realsense_image, desired_encoding="bgr8")

        font = cv2.FONT_HERSHEY_SIMPLEX
        for i, name in enumerate(bbox_list):
            cv2.rectangle(cv2_image,(bb[i].xmin,bb[i].ymin),(bb[i].xmax,bb[i].ymax),(0,255,0),2)
            pix_y = bb[i].ymin-5
            if pix_y<10: pix_y=10
            cv2.putText(cv2_image, name, (bb[i].xmin,pix_y),font,0.5,(0,0,0))
        cv2.imwrite(req.data+"/"+str(time.time())+".png",cv2_image)
        return True

    def listObject(self, request, bb=None, internal_call=False):
        rospy.loginfo('module type : List')

        response_list = RecognitionListResponse()
        coordinate_list = []

        object_name = request.target_name
        sort_option = request.sort_option
        if bb is None:
            bb = RecognitionTools.bbox
        bbox_list = self.createBboxList(bb)

        # 座標を格納したlistを作成
        for i in range(len(bbox_list)):
            if object_name == 'any':
                if not(bbox_list[i] in self.object_dict['any']): continue
            elif object_name != '':
                if not(bbox_list[i] == object_name): continue
            coordinate_list.append([bbox_list[i], [int((bb[i].ymin + bb[i].ymax)/2), int((bb[i].xmin + bb[i].xmax)/2)]])

        # ソート
        if sort_option == 'left':
            coordinate_list.sort(key=lambda x: x[1][1])
        elif sort_option == 'center':
            for i in coordinate_list:
                i[1][1] -= (self.image_width)/2
            coordinate_list.sort(key=lambda x: abs(x[1][1]))
            for i in coordinate_list:
                i[1][1] += (self.image_width)/2
        elif sort_option == 'right':
            coordinate_list.sort(key=lambda x: x[1][1], reverse=True)
        elif sort_option == 'front':
            name_list = set([row[0] for row in coordinate_list])

            localize_req = RecognitionLocalizeRequest()
            localize_req.sort_option.data = 'left'
            depth_list = []

            for name in name_list:
                loop_count = self.countObject(RecognitionCountRequest(name), bb=bb).num
                localize_req.target_name = name
                for i in range(loop_count):
                    localize_req.sort_option.num = i
                    centroid = self.localizeObject(localize_req, bb=bb).point
                    depth_list.append([name, centroid])
            depth_list.sort(key=lambda x: x[1].x)

        try:
            response_list.object_list = depth_list
        except NameError:
            response_list.object_list = coordinate_list

        # serverの呼び出し
        if not internal_call:
            response_list.object_list = [row[0] for row in response_list.object_list]
        return response_list


    
               
 



if __name__ == '__main__':
    rospy.init_node('detect_cloth_color')
    detect_cloth_color = DetectClothColor()
    rospy.spin()
