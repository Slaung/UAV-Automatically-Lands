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
    video_sub = rospy.Subscriber("usb_cam/image_raw" , Image, callback = OD.image_callback)

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
    takeoff_cmd.altitude = 5
    #takeoff_cmd.yaw = 180
    #if(takeoff_client.call(takeoff_cmd)):
    #    rospy.loginfo("Takeoff Success")

    ### Land ###
    land_cmd = CommandTOLRequest()

    for i in range(100):   
        delay_rate.sleep()

    ### Position ###
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 5
    #local_pos_pub.publish(pose)


    ### wait for publish position ###
    for i in range(20):   
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
    
    ### Create log ###
    t_log = []
    area_log = []
    x_speed_log = []
    y_speed_log = []
    z_speed_log = []
    bbox_x_log = []
    bbox_y_log = []
    x_axis_log = []
    y_axis_log = []
    z_axis_log = []
    pred_h_log = []

    ### Controller ###
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
                if(int(float(anaconda_callback_string)) < 1):
                    if(land_client.call(land_cmd)):
                        rospy.loginfo("Trying to land")
                        break
                else:
                    speed.linear.x = PID.x_vel
                    speed.linear.y = PID.y_vel
                speed.linear.z = PID.z_vel

                ### Velocity publish ###
                velocity_pub.publish(speed)
                rospy.loginfo("t: %d", count)
                rospy.loginfo("area: %d", Bbox_area)
                rospy.loginfo("x_speed: %f", PID.x_vel)
                rospy.loginfo("y_speed: %f", PID.y_vel)
                rospy.loginfo("z_speed: %f", PID.z_vel)
                rospy.loginfo("Bbox_x: %f", Bbox_x)
                rospy.loginfo("Bbox_y: %f", Bbox_y)
                rospy.loginfo("x_axis: %f", local_state.pose.position.x)
                rospy.loginfo("y_axis: %f", local_state.pose.position.y)
                rospy.loginfo("z_axis: %f", local_state.pose.position.z)
                rospy.loginfo("pred_h: %s", anaconda_callback_string)
                t_log.append(count)
                area_log.append(Bbox_area)
                x_speed_log.append(PID.x_vel)
                y_speed_log.append(PID.y_vel)
                z_speed_log.append(PID.z_vel)
                bbox_x_log.append(Bbox_x)
                bbox_y_log.append(Bbox_y)
                x_axis_log.append(local_state.pose.position.x)
                y_axis_log.append(local_state.pose.position.y)
                z_axis_log.append(local_state.pose.position.z)
                pred_h_log.append(anaconda_callback_string)

        else:
            rospy.loginfo("failed detection")

        count += 1
        detection_rate.sleep()

    ### Write log ###   
    current_time = rospy.Time.now()
    file_name = "log_{}.txt".format(current_time)

    with open(file_name, 'w') as file:
        for i in range(count-2):
            file.write("t: {}\n".format(t_log[i]))
            file.write("area: {}\n".format(area_log[i]))
            file.write("x_speed: {}\n".format(x_speed_log[i]))
            file.write("y_speed: {}\n".format(y_speed_log[i]))
            file.write("z_speed: {}\n".format(z_speed_log[i]))
            file.write("Bbox_x: {}\n".format(bbox_x_log[i]))
            file.write("Bbox_y: {}\n".format(bbox_y_log[i]))
            file.write("x_axis: {}\n".format(x_axis_log[i]))
            file.write("y_axis: {}\n".format(y_axis_log[i]))
            file.write("z_axis: {}\n".format(z_axis_log[i]))
            file.write("pred_h: {}\n".format(pred_h_log[i]))
