#! /usr/bin/env python


import os
import rospy
import actionlib
import threading
from flir_pantilt_d46.msg import PtuGotoAction
from flir_pantilt_d46.msg import PtuGotoGoal
from trajectory_image_record.msg import RecordImageAction
from trajectory_image_record.msg import RecordImageResult
from trajectory_image_record.observation_recorder import ObservationRecorder


class ObservationRecordManager(object):

    def __init__(self, name):
        self._action_name = name
        self._pan = 0
        self._tilt = 20
        self._pan_vel = 0
        self._tilt_vel = 5
        self._recorder = ObservationRecorder(
            rospy.get_param("~path", "/home/%s/Videos/" % os.uname()[1]),
            rospy.get_param("~ubd_topic", "/upper_body_detector/bounding_box_centres"),
            rospy.get_param("~tracker_topic", "/people_tracker/positions"),
            rospy.get_param("~image_topic", "/head_xtion/rgb/image_raw")
        )
        rospy.loginfo("%s is starting an action server", name)
        self._as = actionlib.SimpleActionServer(
            name, RecordImageAction, execute_cb=self.execute, auto_start=False
        )
        rospy.loginfo("Connecting to SetPtuState action server...")
        self._ac = actionlib.SimpleActionClient(
            "SetPtuState", PtuGotoAction
        )
        self._ac.wait_for_server()
        self._as.start()

    def tilting(self, reverse=False):
        pan = self._pan
        tilt = self._tilt
        if reverse:
            pan *= -1
            tilt *= -1
        self._ac.send_goal(
            PtuGotoGoal(pan, tilt, self._pan_vel, self._tilt_vel)
        )
        self._ac.wait_for_result()
        self._ac.get_result()
        # self._ac.cancel_all_goals()

    def execute(self, goal):
        self.tilting()  # cek apakah ini bisa di cancel gak, klo ternyata bisa dan balik, kodingan perlu dirubah
        thread = threading.Thread(target=self._recorder.record, args=(goal.duration,))
        thread.start()
        while not self._as.is_preempt_requested():
            rospy.sleep(0.1)
        if self._as.is_preempt_requested():
            self._recorder.stop_recording()
            self._as.set_preempted()
        else:
            self._as.set_succeeded(RecordImageResult())
        thread.join()
        self._recorder.save()
        self._recorder.reset()
        self.tilting(True)


if __name__ == '__main__':
    rospy.init_node("observation_record_manager")
    orm = ObservationRecordManager(rospy.get_name())
    rospy.spin()
