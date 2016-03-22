#!/usr/bin/env python

import tf
import yaml
import rospy
import argparse
import threading
import message_filters
from std_msgs.msg import Header
from scipy.spatial.distance import euclidean
from bayes_people_tracker.msg import PeopleTracker
from geometry_msgs.msg import PoseArray, PoseStamped
from trajectory_image_record.video_recorder import VideoRecorder


class ObservationRecorder(object):

    def __init__(self, path, poses_topic, tracker_topic, img_topic):
        self.reset()
        self._img_topic = img_topic
        if path[-1] != '/':
            path += '/'
        self._file_path = path
        self._tfl = tf.TransformListener()
        rospy.loginfo("Subscribe to %s..." % img_topic)
        subs = [
            message_filters.Subscriber(poses_topic, PoseArray),
            message_filters.Subscriber(tracker_topic, PeopleTracker),
        ]
        ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=5, slop=0.15)
        ts.registerCallback(self.cb)

    def reset(self):
        self.stop = False
        self.uuids = list()
        self._ubd_pos = list()
        self._tracker_pos = list()
        self.recorded_info = list()
        self._tracker_uuids = list()

    def cb(self, ubd_cent, pt):
        self._ubd_pos = self.to_world_all(ubd_cent)
        self._tracker_pos = [i.position for i in pt.poses]
        self._tracker_uuids = pt.uuids

    def to_world_all(self, pose_arr):
        transformed_pose_arr = list()
        try:
            fid = pose_arr.header.frame_id
            for cpose in pose_arr.poses:
                ctime = self._tfl.getLatestCommonTime(fid, "/map")
                pose_stamped = PoseStamped(Header(1, ctime, fid), cpose)
                # Get the translation for this camera's frame to the world.
                # And apply it to all current detections.
                tpose = self._tfl.transformPose("/map", pose_stamped)
                transformed_pose_arr.append(tpose.pose.position)
        except tf.Exception as e:
            rospy.logwarn(e)
            # In case of a problem, just give empty world coordinates.
            return []
        return transformed_pose_arr

    def record(self, duration):
        self.stop = False
        self.uuids = list()
        start_time = rospy.Time.now()
        vr = VideoRecorder(
            self._file_path+str(start_time.secs), record_duration=duration,
            img_topic=self._img_topic
        )
        vr.start()
        end_time = rospy.Time.now()
        while not self.stop and (end_time - start_time).secs < duration:
            for i in self._ubd_pos:
                for ind, j in enumerate(self._tracker_pos):
                    if euclidean([i.x, i.y], [j.x, j.y]) < 0.3:
                        if self._tracker_uuids[ind] not in self.uuids:
                            self.uuids.append(self._tracker_uuids[ind])
                            rospy.loginfo(
                                "%d persons have been detected so far..." % len(self.uuids)
                            )
            rospy.sleep(0.1)
            end_time = rospy.Time.now()
        if self.stop:
            rospy.logwarn("Forcing kill the camera recorder...")
            vr.force_kill()
        else:
            vr.join()
        self.recorded_info.append((start_time, end_time, len(self.uuids)))

    def stop_recording(self):
        self.stop = True

    def save(self, file_name=rospy.Time.now().secs):
        stream = file(self._file_path+str(file_name)+'.data', 'a')
        yaml.dump(self.recorded_info, stream)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog="observation_recorder")
    parser.add_argument(
        '-p', dest="poses_topic", default="/upper_body_detector/bounding_box_centres",
        help="Image topic to be stored (default=/upper_body_detector/bounding_box_centres)"
    )
    parser.add_argument(
        "-t", dest="tracker_topic", default="/people_tracker/positions",
        help="Trajectory topic to be stored (default=/people_tracker/positions)"
    )
    parser.add_argument(
        "-i", dest="image_topic", default="/head_xtion/rgb/image_raw",
        help="Image topic to be stored (default=/head_xtion/rgb/image_raw)"
    )
    parser.add_argument(
        'file_path',
        help="A path to store video files e.g. ('/home/user/Videos/')"
    )
    args = parser.parse_args()

    rospy.init_node("observation_recorder")
    orm = ObservationRecorder(
        args.file_path, args.poses_topic, args.tracker_topic, args.img_topic
    )
    orm.record(60)
    rospy.sleep(10)
    thread = threading.Thread(target=orm.record, args=(60,))
    thread.start()
    rospy.sleep(10)
    orm.stop_recording()
    thread.join()
    orm.save()
