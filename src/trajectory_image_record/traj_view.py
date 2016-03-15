#!/usr/bin/env python

import rospy
import argparse
from sensor_msgs.msg import Image
from trajectory_image_record.msg import ImageRecord
from mongodb_store.message_store import MessageStoreProxy
from human_trajectory.traj_visualisation import TrajectoryVisualization


class TrajectoryImageView(object):

    def __init__(self, im_topic, collection_name):
        self._traj_vis = TrajectoryVisualization(
            "trajectory_individual_visualisation", "direction"
        )
        rospy.loginfo("Connect to %s collection..." % collection_name)
        self._traj_db = MessageStoreProxy(collection=collection_name).query(
            ImageRecord._type, sort_query=[("header.stamp.secs", 1)]
        )
        rospy.loginfo("Publish to %s..." % im_topic)
        self._pub = rospy.Publisher(
            im_topic, Image, queue_size=10
        )
        self._rate = rospy.Rate(1)

    def view(self):
        for i in self._traj_db:
            self._pub.publish(i[0].image)
            for uuid in i[0].uuids:
                if uuid in self._traj_vis.trajs.traj.keys():
                    traj = self._traj_vis.trajs.traj[uuid]
                    self._traj_vis.visualize_trajectory(traj)
                    self._rate.sleep()
                    self._traj_vis.delete_trajectory(traj)
                else:
                    rospy.logwarn("%s is not recorded in people_trajectory collection" % uuid)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog="image_view")
    parser.add_argument(
        "collection_name", nargs='?', default="trajectory_images",
        help="Collection name in db message_store (default=trajectory_images)"
    )
    args = parser.parse_args()

    rospy.init_node("trajectory_image_view")
    tiv = TrajectoryImageView(rospy.get_name(), args.collection_name)
    tiv.view()
