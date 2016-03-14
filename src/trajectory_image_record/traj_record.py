#!/usr/bin/env python

import rospy
import argparse
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from human_trajectory.msg import Trajectories
from trajectory_image_record.msg import ImageRecord
from mongodb_store.message_store import MessageStoreProxy


class TrajectoryImageRecord(object):

    def __init__(self, record_topic, img_topic, traj_topic, collection_name):
        self.last_traj_ids = list()
        self._counter = 1
        self._max_size = 100
        self._trajs = Trajectories()
        self._new = False
        self._img = Image()
        self._last_taken = rospy.Time.now().secs
        rospy.loginfo("Starting Trajectory Image Recording...")

        rospy.loginfo("Subscribe to %s..." % traj_topic)
        self._sub_traj = rospy.Subscriber(
            # rospy.get_param(
            #     "~logging_manager_topic", "/logging_manager/log_stamped"
            # ),
            traj_topic, Trajectories, self._traj_cb, None, 10
        )
        rospy.loginfo("Subscribe to %s..." % img_topic)
        self._sub_image = rospy.Subscriber(
            img_topic, Image, self._image_cb, None, 10
        )
        rospy.loginfo("Publish to %s..." % record_topic)
        self._pub = rospy.Publisher(
            record_topic, ImageRecord, queue_size=10
        )
        rospy.loginfo("Store to %s collection..." % collection_name)
        self._store_client = MessageStoreProxy(collection=collection_name)

    def _traj_cb(self, trajs):
        for traj in trajs.trajectories:
            if traj.uuid not in self.last_traj_ids:
                self._trajs = trajs
                self._new = True
                break

    def _image_cb(self, image):
        self._img = image

    def record(self):
        while not rospy.is_shutdown():
            traj_ids = list()
            if self._new:
                for traj in self._trajs:
                    if traj.uuid not in self.last_traj_ids:
                        self.last_traj_ids.append(traj.uuid)
                        self.last_traj_ids = self.last_traj_ids[:-1*self._max_size]
                        traj_ids.append(traj.uuid)
                record = ImageRecord(
                    Header(self._counter, rospy.Time.now(), '/map'),
                    traj_ids, self._img
                )
                rospy.loginfo(
                    "Publish an image for trajectories with uuid: %s" % str(traj_ids)
                )
                self._pub.publish(record)
                self._new = False
                self._store_client.insert(record)
            else:
                temp = rospy.Time.now().secs
                if temp - self._last_taken > 10:
                    record = ImageRecord()
                    record.header = Header(self._counter, rospy.Time.now(), '/map'),
                    self._store_client.insert(record)
                    self._last_taken = temp


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog="image_record")
    parser.add_argument(
        "image_topic", default="/head_xtion/rgb/image_raw/compressed_rgb",
        help="Image topic to be stored (default=/head_xtion/rgb/image_raw/compressed_rgb)"
    )
    parser.add_argument(
        "trajectory_topic", default="/human_trajectories/trajectories/batch",
        help="Trajectory topic to be stored (default=/human_trajectories/trajectories/batch)"
    )
    parser.add_argument(
        "collection_name", default="trajectory_images",
        help="Collection name in db message_store (default=trajectory_images)"
    )
    args = parser.parse_args()
    rospy.init_node("trajectory_image_record")
    tir = TrajectoryImageRecord(
        rospy.get_name(), args.image_topic,
        args.trajectory_topic, args.collection_name
    )
    tir.record()
