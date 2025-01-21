#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from morai_msgs.msg import CtrlCmd

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    """
    m/s 단위 속도(v)와 rad/s 단위 각속도(omega), 휠베이스(wheelbase)를 입력받아
    Ackermann 방정식에 따라 앞바퀴 조향각(라디안)을 계산하는 함수
    """
    if omega == 0 or v == 0:
        return 0.0
    # 회전 반경 = v / omega
    radius = v / omega
    # 조향각 = atan( wheelbase / 회전반경 )
    return math.atan(wheelbase / radius)

def cmd_vel_callback(twist_msg):
    """
    /mppi/cmd_vel (Twist) 메시지를 수신하는 콜백 함수
    1) Ackermann 조향각 계산
    2) 속도(m/s)를 km/h로, 조향각(라디안)을 deg로 변환
    3) CtrlCmd 메시지 생성 및 퍼블리시
    """
    # 1) Ackermann 조향각 (rad)
    v = twist_msg.linear.x  # m/s
    omega = twist_msg.angular.z  # rad/s
    steering_rad = convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase)

    # 2) 단위 변환
    # m/s -> km/h
    velocity_kmh = v * 3.6
    # rad -> deg (필요에 따라 부호를 바꿀 수 있음. 예: -math.degrees(...) )
    steering_deg = math.degrees(steering_rad)

    # 3) CtrlCmd 메시지 생성
    ctrl_cmd_msg = CtrlCmd()
    ctrl_cmd_msg.velocity = velocity_kmh
    ctrl_cmd_msg.steering = -steering_deg

    # 퍼블리시
    speed_pub.publish(ctrl_cmd_msg)

if __name__ == "__main__":
    rospy.init_node("speedpub", anonymous=False)

    # 파라미터로부터 wheelbase 로드 (기본값 1.0)
    wheelbase = rospy.get_param("~wheelbase", 0.7)

    # 퍼블리셔: /lp_ctrl (CtrlCmd 메시지)
    speed_pub = rospy.Publisher('/lp_ctrl', CtrlCmd, queue_size=1)

    # 서브스크라이버: /mppi/cmd_vel (Twist 메시지)
    rospy.Subscriber('/mppi/cmd_vel', Twist, cmd_vel_callback)

    rospy.loginfo("Node [speedpub] started. Subscribing [/mppi/cmd_vel], Publishing [/lp_ctrl]. Wheelbase = %.2f" % wheelbase)

    rospy.spin()
