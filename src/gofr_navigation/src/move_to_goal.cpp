#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include "sound_play/sound_play.h"

// std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);
char choose();

/** declare the coordinates of interest **/
double xCafe = 15.50;
double yCafe = 10.20;
double xOffice1 = 27.70 ;
double yOffice1 = 12.50;
double xOffice2 = 30.44 ;
double yOffice2 = 12.50;
double xOffice3 = 35.20 ;
double yOffice3 = 13.50;

bool goalReached = false;
 int main(int argc, char** argv){
   ros::init(argc, argv, "map_navigation_node");
   ros::NodeHandle n;
   //sound_play::SoundClient sc;
   ros::spinOnce();
   //path_to_sounds = "/home/ros/catkin_ws/src/gaitech_edu/src/sounds/";

   char choice = 'q';
   do{
      choice =choose();
      if (choice == '0'){
         goalReached = moveToGoal(xCafe, yCafe);
      }else if (choice == '1'){
         goalReached = moveToGoal(xOffice1, yOffice1);
      }else if (choice == '2'){
         goalReached = moveToGoal(xOffice2, yOffice2);
      }else if (choice == '3'){
         goalReached = moveToGoal(xOffice3, yOffice3);
      }
      if (choice!='q'){
         if (goalReached){
            ROS_INFO("Congratulations!");
            ros::spinOnce();
           //sc.playWave(path_to_sounds+"ship_bell.wav");
            ros::spinOnce();

         }else{
            ROS_INFO("Hard Luck!");
            //sc.playWave(path_to_sounds+"short_buzzer.wav");
         }
      }
   }while(choice !='q');
   return 0;
}