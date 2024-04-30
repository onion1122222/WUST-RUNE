import rospy
from geometry_msgs.msg import Point
import numpy as np
import datetime
import math
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter

# 定义计算符文速度的函数
def RuneSpeedFunc(t, a, omega, c):
    return a * np.sin(omega * (t + c)) + (2.09 - a)

# 定义计算时间间隔内的角度变化的函数
def CalcDeltaRunePos(t0, t1, a, omega, c):
    p0 = -a / omega * np.cos(omega * (t0 + c)) + (2.090 - a) * t0
    p1 = -a / omega * np.cos(omega * (t1 + c)) + (2.090 - a) * t1
    return p1 - p0
class RuneDetector:
    def __init__(self):
        self.Reset()

    def Reset(self):
        self.result_img_ = None
        self.binary_img_ = None
        self.small_lights_ = []
        self.big_lights_ = []
        self.huge_lights_ = []
        self.circular_lights_ = []
        self.runes_ = []
        self.center_ = np.array([0,0])
        self.target_center_ = np.array([0,0])
        self.target_angles_ = []
        self.key_angles_ = []
        self.predict_angles_ = []
        self.predict_diffs_ = []
        self.rune_speeds_ = []
        self.rune_speeds_smoothed_ = []
        self.time_stamps_ = []
        self.start_time_ = datetime.datetime.now()
        self.lose_target_count = 0
        self.rune_speed_params = [1.0, 1.9, 1.0]

    def UpdateTargetPosition(self, point):
        self.target_center_ = np.array([point.x, point.y])
        self.CollectData()

    def CalcTargetAngle(self, pt):
        dy = pt[1] - self.center_[1]
        dx = pt[0] - self.center_[0]
        angle = math.atan2(dy, dx)
        if angle < 0:
            angle += math.pi * 2
        return angle

    def CollectData(self):
        target_angle = self.CalcTargetAngle(self.target_center_)
        self.target_angles_.append(target_angle)
        if len(self.target_angles_) >= 2:
            rune_speed = 50 * (self.target_angles_[-1] - self.target_angles_[-2])
            if abs(rune_speed) > 2:
                rune_speed = self.rune_speeds_[-1]
            self.rune_speeds_.append(rune_speed)
            self.rune_speeds_smoothed_.append(rune_speed)
            self.time_stamps_.append((datetime.datetime.now() - self.start_time_).total_seconds())
        self.Filter()
        if len(self.target_angles_) >= 50:
            self.FitTraceCurve()

    def FitTraceCurve(self):
        t = np.array(self.time_stamps_)
        y = np.array(self.rune_speeds_smoothed_)
        p0 = [0.9, 1.9, 0.0]
        try:
            popt, pcov = curve_fit(RuneSpeedFunc, t, y, p0)
            self.rune_speed_params = popt
            print('拟合参数：', popt)
        except Exception as e:
            print('拟合失败', e)

    def Filter(self):
        if len(self.rune_speeds_smoothed_) < 5:
            return
        y = np.array(self.rune_speeds_smoothed_)
        y_smooth = savgol_filter(y, 5, 3)
        self.rune_speeds_smoothed_ = list(y_smooth)

    def Predict(self, dt: float):
        self.CollectData()
        
        a, omega, c = self.rune_speed_params
        t0 = self.time_stamps_[-1]
        t1 = t0 + dt
        dp = CalcDeltaRunePos(t0, t1, a, omega, c)
        
        rotation_radius = np.linalg.norm(self.target_center_ - self.center_)
        if rotation_radius < 0.1:
            return None
        
        predict_pos_angle = self.target_angles_[-1] + dp
        if predict_pos_angle > math.pi * 2:
            predict_pos_angle -= math.pi * 2
        self.predict_angles_.append(predict_pos_angle)
        
        return predict_pos_angle

class RuneDetectorSubscriber:
    def __init__(self):
        rospy.init_node('rune_detector_subscriber', anonymous=True)
        self.detector = RuneDetector()
        self.subscriber = rospy.Subscriber("target_position", Point, self.position_callback)

    def position_callback(self, msg):
        print(f"接收到目标位置 - x: {msg.x}, y: {msg.y}")
        self.detector.UpdateTargetPosition(msg)

if __name__ == '__main__':
    try:
        subscriber = RuneDetectorSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
