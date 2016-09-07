#include <iostream>
#include <ros/ros.h>
#include <termios.h>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>

#define KEYCODE_D 0x64 
#define KEYCODE_A 0x61
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_C 0x63
#define KEYCODE_H 0x68
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_SPACE 0x20

void keyLoop(ros::Publisher &t_pub)
{
    char c;
    bool dirty = false;
    int kfd = 0;
    struct termios cooked, raw;

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    double x_pos = 0.0;
    double y_pos = 0.0;
    double z_pos = 1.0;
    double desired_yaw = 90.0;

    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the drone.");

    trajectory_msg.header.stamp = ros::Time::now();
    Eigen::Vector3d first_position(x_pos, y_pos, z_pos);
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(first_position,
      desired_yaw, &trajectory_msg);
    t_pub.publish(trajectory_msg);
    std::cout << "Moving to (" << x_pos << ", " << y_pos << ", " << z_pos << ")" << std::endl;

    for(;;)
    {
        // get the next event from the keyboard  
        if(read(kfd, &c, 1) < 0)
        {
          perror("read():");
          exit(-1);
        }

        ROS_DEBUG("value: 0x%02X\n", c);
      
        switch(c)
        {
          case KEYCODE_A:
            ROS_DEBUG("LEFT");
            puts("LEFT");
            x_pos += -1.0;
            dirty = true;
            break;
          case KEYCODE_D:
            ROS_DEBUG("RIGHT");
            puts("RIGHT");
            x_pos += 1.0;
            dirty = true;
            break;
          case KEYCODE_W:
            ROS_DEBUG("FORWARD");
            puts("FORWARD");
            y_pos += 1.0;
            dirty = true;
            break;
          case KEYCODE_S:
            ROS_DEBUG("BACKWARD");
            puts("BACKWARD");
            y_pos += -1.0;
            dirty = true;
            break;

          case KEYCODE_C:
            ROS_DEBUG("DOWN");
            puts("DOWN");
            if(z_pos - 1.0 >= 1.0){
                z_pos -= 1.0;
                dirty = true;
            }
            else
                dirty = false;
            break;

          case KEYCODE_SPACE:
            ROS_DEBUG("UP");
            puts("UP");
            if(z_pos + 1.0 < 50){
                z_pos += 1.0;
                dirty = true;
            }
            else{
                dirty = false;
            }
            break;

          case KEYCODE_H:
            ROS_DEBUG("HOME");
            puts("HOME");
            x_pos = 0.0;
            y_pos = 0.0;
            z_pos = 1.0;
            desired_yaw = 90.0;
            dirty = true;
            break;

          case KEYCODE_Q:
            ROS_DEBUG("YAW LEFT");
            puts("YAW LEFT");
            desired_yaw += -10.0;
            dirty = true;
            break;

          case KEYCODE_E:
            ROS_DEBUG("YAW RIGHT");
            puts("YAW RIGHT");
            desired_yaw += 10.0;
            dirty = true;
            break;

          default:
            dirty = false;
            break;
        }
       
        if(dirty){
            trajectory_msg.header.stamp = ros::Time::now();
            Eigen::Vector3d desired_position(x_pos, y_pos, z_pos);
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
              desired_yaw, &trajectory_msg);
            t_pub.publish(trajectory_msg);
            std::cout << "Moving to (" << x_pos << ", " << y_pos << ", " << z_pos << ", " << desired_yaw << ")" << std::endl;
            dirty = false;
        }
        ros::spinOnce();
    }


    return;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "key");
    ros::NodeHandle nh_;
    ros::Publisher trajectory_pub =
    nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
    "/firefly/command/trajectory", 10);

    keyLoop(trajectory_pub);

    ros::spin();
  
    return 0;
}

