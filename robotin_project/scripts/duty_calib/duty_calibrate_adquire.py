import rospy
import pickle    
import threading

from robotin_project.msg import TEL

samples_wheel1 = []
samples_wheel2 = []
samples_wheel3 = []
samples_wheel4 = []

def timeout():
    global samples_wheel1,samples_wheel2,samples_wheel3,samples_wheel4

    with open('calibration_wheel1.dat','wb') as fp:
        pickle.dump(samples_wheel1,fp)
    with open('calibration_wheel2.dat','wb') as fp:
        pickle.dump(samples_wheel2,fp)
    with open('calibration_wheel3.dat','wb') as fp:
        pickle.dump(samples_wheel3,fp)
    with open('calibration_wheel4.dat','wb') as fp:
        pickle.dump(samples_wheel4,fp)

    print("Model save")

def callback_wheel1(data):
    global timer
    samples_wheel1.append([data.current_velocity,data.demanded_duty])
    timer.cancel()
    timer = threading.Timer(5,timeout)
    timer.start()
def callback_wheel2(data):
    samples_wheel2.append([data.current_velocity,data.demanded_duty])
def callback_wheel3(data):
    samples_wheel3.append([data.current_velocity,data.demanded_duty])
def callback_wheel4(data):
    samples_wheel4.append([data.current_velocity,data.demanded_duty])

if __name__=="__main__":    
    print("Process")

    rospy.init_node("duty_calibrate_process")
    rospy.Subscriber("/wheel1/tel_vel",TEL,callback_wheel1)
    rospy.Subscriber("/wheel2/tel_vel",TEL,callback_wheel2)
    rospy.Subscriber("/wheel3/tel_vel",TEL,callback_wheel3)
    rospy.Subscriber("/wheel4/tel_vel",TEL,callback_wheel4)

    timer = threading.Timer(5,timeout)
    timer.start()

    rospy.spin()