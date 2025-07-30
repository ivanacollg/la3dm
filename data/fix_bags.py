#!/usr/bin/env python

import rosbag

input_bag = 'sim_unstructured/sim_unstructured.bag'
output_bag = 'sim_unstructured_fixed.bag'
old_frame = '/map'
new_frame = 'map'

with rosbag.Bag(output_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        # Update frame_id if the message has a header
        if hasattr(msg, 'header') and msg.header.frame_id == old_frame:
            msg.header.frame_id = new_frame
        # Also check embedded headers, e.g., in PoseStamped arrays, etc.
        if hasattr(msg, 'poses'):
            for pose in msg.poses:
                if hasattr(pose, 'header') and pose.header.frame_id == old_frame:
                    pose.header.frame_id = new_frame
        outbag.write(topic, msg, t)
