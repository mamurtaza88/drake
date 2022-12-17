#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//Pipe files
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

//Thread
#include <thread>


#include "geometry_msgs/Twist.h"

// double arr[6];
double pub_arr[6];
ros::Publisher arr_pub;

void getData()
{
  double arr[6];
  // std::cout << "getData Check 1 \n";
  int fd;
  fd = open("/home/sfam/Documents/GitHub/drake/catkin_ws/src/wrist_tracking/scripts/end_effector_info", O_RDONLY);

  do {
    if (read(fd, arr, sizeof(double)*6) == -1) {
      // std::cout << "getData Check 2 \n";
      // std::cout << "error reading file!  fd = " << fd << std::endl;
    }
    else {
      // std::cout << "getData Check 3 \n";
      // std::cout << "Pose Reading" << std::endl;
      for (int i = 0; i < 6; i++) {
        // std::cout << arr[i] << std::endl;
        pub_arr[i] = arr[i];
        // ee_velocity[i] = arr[i + 6];
      }
    }
 
    
  } while (true);
}

void sendData()
{
  // std::cout << "sendData Check 1 \n";  
  geometry_msgs::Twist arr_msg;
  while(true)
  {
    // std::cout << "sendData Check 2 \n";
    arr_msg.linear.x = pub_arr[0];
    arr_msg.linear.y = pub_arr[1];
    arr_msg.linear.z = pub_arr[2];
    arr_msg.angular.x = pub_arr[3];
    arr_msg.angular.y = pub_arr[4];
    arr_msg.angular.z = pub_arr[5];
    std::cout << "Sent \n";
    arr_pub.publish(arr_msg);
  }

}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "kuka2_Twist");
  // std::cout << "main Check 1 \n";  

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
    // std::cout << "main Check 2 \n";  

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/ridgeback2/chatter", 1000);
  arr_pub = n.advertise<geometry_msgs::Twist>("ridgeback2/arr_val2", 1);
  // std::cout << "main Check 3 \n";  

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  //Pipe Stuff
  int fd;
  //  std::cout << "main Check 4 \n";  

  int read_bytes;
  std::string s = "/home/sfam/Documents/GitHub/drake/catkin_ws/src/wrist_tracking/scripts/end_effector_info";

  mkfifo("/home/sfam/Documents/GitHub/drake/catkin_ws/src/wrist_tracking/scripts/end_effector_info",0666);
  // mkfifo(s,0666);
    // std::cout << "main Check 5 \n";  

  fd = open("/home/sfam/Documents/GitHub/drake/catkin_ws/src/wrist_tracking/scripts/end_effector_info", O_RDONLY);
  // geometry_msgs::Twist arr_msg;

  // std::cout << "main Check 6 \n";  

  std::thread data_recv_thrd(getData); 
    // std::cout << "main Check 7 \n";  
 
  std::thread send_cmd_thrd(sendData);

  data_recv_thrd.join();
  send_cmd_thrd.join();
  
    // while (ros::ok())
  // {
  //   /**
  //    * This is a message object. You stuff it with data, and then publish it.
  //    */

  //   read_bytes = read(fd, arr, sizeof(double)*6);
  //   std_msgs::String msg;

  //   std::stringstream ss;
  //   ss << "hello world " << count;
  //   msg.data = ss.str();

  //   // ROS_INFO("%s", msg.data.c_str());
  //   ROS_INFO("arr = ");
  //   for(int i = 0; i < 6;i++)
  //   {
  //     std::cout << "arr[" << i << "] = " << arr[i] << std::endl;
  //     // ROS_INFO("%d ", arr[i]);
  //   }
    
  //   arr_msg.linear.x = arr[0];
  //   arr_msg.linear.y = arr[1];
  //   arr_msg.linear.z = arr[2];
  //   arr_msg.angular.x = arr[3];
  //   arr_msg.angular.y = arr[4];
  //   arr_msg.angular.z = arr[5];

  //   /**
  //    * The publish() function is how you send messages. The parameter
  //    * is the message object. The type of this object must agree with the type
  //    * given as a template parameter to the advertise<>() call, as was done
  //    * in the constructor above.
  //    */
  //   chatter_pub.publish(msg);
  //   arr_pub.publish(arr_msg);

  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   ++count;
  // }


  return 0;
}
