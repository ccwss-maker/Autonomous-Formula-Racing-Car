#include "ros/ros.h"
#include <kbd_ros_msgs/kbd.h>
#include <sstream>
#include <linux/input.h>
#include <fcntl.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
//cat /proc/bus/input/devices
//sudo chmod 777 /dev/input/event6
//P: Phys=usb-3610000.xhci-2.4/input0
//#define DEV_PATH "/dev/input/event5"   //difference is possible

typedef struct
{
    int w=0;
    int s=0;
    int a=0;
    int d=0;
}keyboard_;

kbd_ros_msgs::kbd scanKeyboard(int keys_fd)
{
	static struct input_event t;
    static kbd_ros_msgs::kbd keyboard;
    if(read(keys_fd, &t, sizeof(t)) == sizeof(t))
    {
        if(t.type==EV_KEY)
        {
            if(t.value==0 || t.value==1)
            {
                switch(t.code)
                {
                    case 17:
                        keyboard.w=t.value;
                        break;
                    case 31:
                        keyboard.s=t.value;
                        break;
                    case 30:
                        keyboard.a=t.value;
                        break;
                    case 32:
                        keyboard.d=t.value;
                        break;
                }
                //ROS_INFO("key %d %s\n", t.code, (t.value) ? "Pressed" : "Released");
            }
        }
    }
	return keyboard;
}

int main(int argc, char **argv)
{
    YAML::Node config = YAML::LoadFile("/home/wust/Personal_Data/ros_ws/robot-my_ws/src/robot/robot_program/config/keyboard.yaml");
    std::string PATH =config["path"].as<std::string>();
    int keys_fd;
	keys_fd=open(PATH.c_str(), O_RDONLY);
	if(keys_fd <= 0)
	{
		ROS_INFO("open /dev/input/event6 device error!\n");
		return -1;
	}
    else
    {
        ROS_INFO("keyboard done\n");
    }

    ros::init(argc, argv, "kbdmsg");
    ros::NodeHandle n;
    ros::Publisher kbd_pub = n.advertise<kbd_ros_msgs::kbd>("keyboard", 1);

    //ros::Rate loop_rate(50);

    while (ros::ok())
    {
        kbd_ros_msgs::kbd msg;
        msg=scanKeyboard(keys_fd);
        kbd_pub.publish(msg);
        ros::spinOnce();
        //loop_rate.sleep();
    }

    close(keys_fd);
    ros::shutdown();
    return 0;
}