#! /usr/bin/env python

import rospy
import cv2 as cv
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from object_detection import ObjectDetection
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from pid_controller import PID_Controller
from std_msgs.msg import String
from collections import Counter
from queue import Queue

### Height queue ###
height_queue = Queue()

current_state = State()
def state_callback(msg):
    global current_state
    current_state = msg

bbox_state = BoundingBoxes()
def bbox_callback(msg):
    global bbox_state
    bbox_state = msg

local_state = PoseStamped()
def local_callback(msg):
    global local_state
    local_state = msg

anaconda_callback_string = ""
def anaconda_callback(data):
    global anaconda_callback_string
    anaconda_callback_string = data.data

# bbox_count_state = ObjectCount()
# def bbox_count_callback(msg):
#     global bbox_count_state
#     bbox_count_state = msg

if __name__ == "__main__":
    rospy.init_node("object_detection_velocity_py")

    ### Camera ###
    OD = ObjectDetection()
    video_sub = rospy.Subscriber("roscam/cam/image_raw" , Image, callback = OD.image_callback)

    ### Darknet ###
    bbox_sub = rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, callback = bbox_callback)

    #bbox_count_sub = rospy.Subscriber("darknet_ros/count", ObjectCount, callback = bbox_count_callback)

    object_detection_img_pub = rospy.Publisher('/object_detection_img', Image, queue_size = 10)

    ### Ros and Anaconda connect ###
    ros_detection_area_pub = rospy.Publisher("ros_detection_area", String, queue_size = 10)

    anaconda_predict_height_sub = rospy.Subscriber("anaconda_predict_height", String, callback = anaconda_callback)
    
    ### Mavros ###
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_callback)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 10)

    velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size = 10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/takeoff")
    takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)

    rospy.wait_for_service("/mavros/cmd/land")
    land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)   

    local_sub = rospy.Subscriber("mavros/local_position/pose" , PoseStamped, callback = local_callback)

    ### hmarker ###
    hmarker_pub = rospy.Publisher("/hmarker2/cmd_vel", Twist, queue_size=10)

    delay_rate = rospy.Rate(20)
    detection_rate = rospy.Rate(5)

    ### Wait for Flight Controller connection ##
    while(not rospy.is_shutdown() and not current_state.connected):
        delay_rate.sleep()

    ### GUIDED ###
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'GUIDED'
    if(set_mode_client.call(offb_set_mode).mode_sent == True):
        rospy.loginfo("GUIDED enabled")

    ### Unlock ###
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    if(arming_client.call(arm_cmd).success == True):
        rospy.loginfo("Vehicle armed")

    ### Take off ###
    takeoff_cmd = CommandTOLRequest()
    takeoff_cmd.altitude = 3
    takeoff_cmd.yaw = 180
    if(takeoff_client.call(takeoff_cmd)):
        rospy.loginfo("Takeoff Success")

    ### Land ###
    land_cmd = CommandTOLRequest()

    for i in range(100):   
        delay_rate.sleep()

    ### Position ###
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 10
    local_pos_pub.publish(pose)


    ### wait for publish position ###
    for i in range(140):   
        delay_rate.sleep()

    ### Velocity controller ###
    speed = Twist()
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    velocity_pub.publish(speed)

    ### Hmarker move controller ###
    # hmarker_cmd = Twist()

    ### PID controller ###
    PID = PID_Controller()

    count = 0
    rospy.loginfo("Object detect...")
    while(not rospy.is_shutdown()):
        ### Camera image ###
        object_detection_img_pub.publish(OD.bridge.cv2_to_imgmsg(OD.frame, "bgr8"))
        frame_h, frame_w, frame_c = OD.frame.shape
        #rospy.loginfo("Camera center point(x, y): %d %d", frame_w/2, frame_h/2)

        if(len(bbox_state.bounding_boxes) > 0):
            ### Bbox image ###
            Bbox_x = (bbox_state.bounding_boxes[0].xmax - bbox_state.bounding_boxes[0].xmin)/2 + bbox_state.bounding_boxes[0].xmin
            Bbox_y = (bbox_state.bounding_boxes[0].ymax - bbox_state.bounding_boxes[0].ymin)/2 + bbox_state.bounding_boxes[0].ymin
            Bbox_area = (bbox_state.bounding_boxes[0].xmax - bbox_state.bounding_boxes[0].xmin) * (bbox_state.bounding_boxes[0].ymax - bbox_state.bounding_boxes[0].ymin)
            #rospy.loginfo("Bbox area:%d", Bbox_area)
            #rospy.loginfo("bbox: %s %d %d %d %d %d", bbox_state.bounding_boxes[0].Class ,bbox_state.bounding_boxes[0].id, bbox_state.bounding_boxes[0].xmin, bbox_state.bounding_boxes[0].ymin, bbox_state.bounding_boxes[0].xmax, bbox_state.bounding_boxes[0].ymax)

            ### Anaconda: height predictor ###
            ros_publish_string = str(Bbox_area)
            ros_detection_area_pub.publish(ros_publish_string)

            ### Wait anaconda inference ###
            if(anaconda_callback_string != ""):
                anaconda_height =  float(anaconda_callback_string)

                ### PID controller ###
                PID.local_pos_control(Bbox_x, Bbox_y, anaconda_height)
                
                ### Set linear speed ###
                if(Bbox_area > 87251):
                    if(land_client.call(land_cmd)):
                        rospy.loginfo("Trying to land")
                        break
                else:
                    speed.linear.x = PID.x_vel
                    speed.linear.y = PID.y_vel
                speed.linear.z = PID.z_vel

                ### Velocity publish ###
                velocity_pub.publish(speed)
                #rospy.loginfo("t:%d, area: %d, z_speed: %f", count, Bbox_area, PID.z_vel)
                #rospy.loginfo("height: %f, pred_h: %s", local_state.pose.position.z, anaconda_callback_string)
                rospy.loginfo("t: %d", count)
                rospy.loginfo("area: %d", Bbox_area)
                rospy.loginfo("z_speed: %f", PID.z_vel)
                rospy.loginfo("real_h: %f", local_state.pose.position.z)
                rospy.loginfo("pred_h: %s", anaconda_callback_string)
        else:
            rospy.loginfo("failed detection")

       
        #rospy.loginfo("velocity =  %f %f %f", PID.x_vel, PID.y_vel, PID.z_vel)

        ### hmarker move ###
        # hmarker_cmd.linear.x = 0.5
        # hmarker_pub.publish(hmarker_cmd)

        #rospy.loginfo("t: %d", count)
        

        count += 1
        detection_rate.sleep()
