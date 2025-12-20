#!/usr/bin/env python3
import rospy
import math
import numpy as np
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

def get_circle_state(t, radius, speed, z_height, phase_offset):
    """
    Returns position, velocity, acceleration, and yaw for a circular trajectory.
    """
    omega = speed / radius
    theta = omega * t + phase_offset
    
    # Position
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    z = z_height
    
    # Velocity
    vx = -radius * omega * math.sin(theta)
    vy = radius * omega * math.cos(theta)
    vz = 0.0
    
    # Acceleration
    ax = -radius * (omega**2) * math.cos(theta)
    ay = -radius * (omega**2) * math.sin(theta)
    az = 0.0
    
    # Yaw (facing tangent to the circle)
    yaw = theta + math.pi / 2.0
    
    return (x, y, z), (vx, vy, vz), (ax, ay, az), yaw

def swarm_circle_demo():
    rospy.init_node('swarm_circle_demo_controller')
    
    num_drones = 3
    radius = 3.0
    speed = 1.5
    z_height = 1.0
    rate_hz = 50.0
    
    pubs = []
    for i in range(num_drones):
        topic = f'/drone_{i}_planning/pos_cmd'
        pub = rospy.Publisher(topic, PositionCommand, queue_size=1)
        pubs.append(pub)
        rospy.loginfo(f'Publisher created for {topic}')
        
    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now()
    
    rospy.loginfo("Starting Swarm Circle Demo...")
    
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        
        for i in range(num_drones):
            phase = i * (2 * math.pi / num_drones)
            pos, vel, acc, yaw = get_circle_state(elapsed, radius, speed, z_height, phase)
            
            cmd = PositionCommand()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "world"
            
            cmd.position.x = pos[0]
            cmd.position.y = pos[1]
            cmd.position.z = pos[2]
            
            cmd.velocity.x = vel[0]
            cmd.velocity.y = vel[1]
            cmd.velocity.z = vel[2]
            
            cmd.acceleration.x = acc[0]
            cmd.acceleration.y = acc[1]
            cmd.acceleration.z = acc[2]
            
            cmd.yaw = yaw
            # yaw_dot is optional/derived usually, but we could set it to omega
            
            pubs[i].publish(cmd)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        swarm_circle_demo()
    except rospy.ROSInterruptException:
        pass