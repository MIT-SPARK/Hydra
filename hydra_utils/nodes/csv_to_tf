#!/usr/bin/env python
"""Node that broadcasts a CSV as a tf."""
import rospy
import sys
import tf2_ros
import geometry_msgs.msg


def main():
    rospy.init_node("csv_to_tf")
    br = tf2_ros.TransformBroadcaster()

    trajectory_file = rospy.get_param("~trajectory_file", "")
    if trajectory_file == "":
        rospy.logfatal("file required!")
        sys.exit(1)

    parent_frame = rospy.get_param("~parent_frame", "world")
    child_frame = rospy.get_param("~child_frame", "left_cam_pgo")

    delta_t = rospy.Duration(-0.05)
    have_first_line = False

    r = rospy.Rate(50)

    with open(trajectory_file, "r") as fin:
        for line in fin:
            if not have_first_line:
                have_first_line = True
                continue

            values = [x.strip() for x in line.split(",")]
            time_ns = int(values[0])
            pose = [float(x) for x in values[1:]]

            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time(0, time_ns)
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = pose[0]
            t.transform.translation.y = pose[1]
            t.transform.translation.z = pose[2]
            t.transform.rotation.w = pose[3]
            t.transform.rotation.x = pose[4]
            t.transform.rotation.y = pose[5]
            t.transform.rotation.z = pose[6]

            while not rospy.is_shutdown():
                if rospy.Time.now() - t.header.stamp > delta_t:
                    break
                r.sleep()

            rospy.logdebug("sending " + str(t.header.stamp.to_sec()))
            br.sendTransform(t)


if __name__ == "__main__":
    main()
