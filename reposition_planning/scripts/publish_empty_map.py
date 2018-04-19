#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

if __name__ == '__main__':
    rospy.init_node('empty_map_publisher')

    publisher = rospy.Publisher('map', OccupancyGrid, queue_size = 1)

    while not rospy.is_shutdown():
        width = 500
        height = 500
        res = 0.05
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = rospy.Time.now()
        grid.info.map_load_time = rospy.Duration(0.0)
        grid.info.resolution = res
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = -0.5 * res * width
        grid.info.origin.position.y = -0.5 * res * height
        grid.info.origin.orientation.w = 1.0
        grid.data = [0 for i in range(width * height)]
        publisher.publish(grid)
        rospy.sleep(1.0)
