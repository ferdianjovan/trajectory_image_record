#!/usr/bin/env python

import os
import rospy
import threading
import subprocess


class VideoRecorder(threading.Thread):

    def __init__(self, file_path_name, img_topic="/head_xtion/rgb/image_color", record_duration=180):
        threading.Thread.__init__(self)
        self.timeout = None
        self.fname = file_path_name
        self.img_topic = img_topic
        self.duration = record_duration
        self.FNULL = open(os.devnull, 'w')

    def force_kill(self):
        self.finish(self)

    def finish(self, _):
        rospy.loginfo("Killing the video recorder...")
        self.proc.kill()
        rospy.loginfo("Recording is done")
        if self.timeout is not None:
            self.timeout.shutdown()

    def run(self):
        rospy.loginfo("Recording is being started...")
        self.proc = subprocess.Popen(
            [
                'rosrun', 'image_view', 'video_recorder', 'image:='+self.img_topic,
                '_filename:=%s.avi' % (self.fname), '_fps:=30'
            ],
            shell=False, stdout=self.FNULL, stderr=subprocess.STDOUT
        )
        self.timeout = rospy.Timer(rospy.Duration(self.duration), self.finish, True)
        self.proc.communicate()
