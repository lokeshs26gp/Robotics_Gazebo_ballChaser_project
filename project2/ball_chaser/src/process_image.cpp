#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

struct Vector2Int
{
  int x;
  int y;
};

ros::ServiceClient client;

Vector2Int previousPosition = {-1,-1};
void drive_robot(float lin_x,float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv))
    {
        ROS_ERROR("failed to call service drive_robot");
    }

}

void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    Vector2Int first_white_pixel_Position = {-1,-1};
    Vector2Int last_white_pixel_Position = {-1,-1};
      
    for(int r=0;r<img.height;r++)
	 {
        for(int c=0;c<img.step;c+=3)
        {
            int pixel_index = c+r*img.step;

            if(img.data[pixel_index] == white_pixel && 
            img.data[pixel_index+1] == white_pixel && 
            img.data[pixel_index+2] == white_pixel)
            {
             
                if(first_white_pixel_Position.x <0 && first_white_pixel_Position.y<0)
                {
                    first_white_pixel_Position = {c,r};
                }
                else 
                  last_white_pixel_Position = {c,r};
            }
        }
    }

   if(first_white_pixel_Position.x >=0 && first_white_pixel_Position.y>=0)
   {
      Vector2Int mid_white_pixel_position = first_white_pixel_Position;
      if(last_white_pixel_Position.x>=0 && last_white_pixel_Position.y>=0)
      {
        int mid_x = (int)((last_white_pixel_Position.x + first_white_pixel_Position.x)/2);
        int mid_y = (int)((last_white_pixel_Position.y + first_white_pixel_Position.y)/2);
        mid_white_pixel_position = {mid_x,mid_y};

      }
      int camera_mid_x = (int)(img.step/2);
      int image_div_left_portion = (int)(img.step/3);
      int image_div_right_portion = img.step - image_div_left_portion;
      float linear_x =0.0f;
      float angular_z = 0.0f;
      if(mid_white_pixel_position.x <image_div_left_portion)//left
      {
         angular_z =  (float)(camera_mid_x - mid_white_pixel_position.x)/(float)camera_mid_x;
        //ROS_INFO_STREAM( "Left - +Angular_Z:"+std::to_string(angular_z) + "("+std::to_string(image_div_left_portion)+"-->"+std::to_string(mid_white_pixel_position.x)+"-->"+std::to_string(image_div_right_portion)+")");
      }
      else if(mid_white_pixel_position.x >image_div_right_portion)    //right   
      {
        angular_z = (float)(camera_mid_x - mid_white_pixel_position.x)/(float)camera_mid_x;
        //ROS_INFO_STREAM( "Right - +Angular_Z:"+std::to_string(angular_z) + "("+std::to_string(image_div_left_portion)+"-->"+std::to_string(mid_white_pixel_position.x)+"-->"+std::to_string(image_div_right_portion)+")");
      }
      else
      {

        linear_x = (float)(img.height - mid_white_pixel_position.y)/(float)img.height;
        //ROS_INFO_STREAM( "Forward - +linear_x:"+std::to_string(linear_x) + "("+std::to_string(image_div_left_portion)+"-->"+std::to_string(mid_white_pixel_position.x)+"-->"+std::to_string(image_div_right_portion)+")");
        
      }

      drive_robot(linear_x,angular_z);
   }
   else 
   {
       // ROS_INFO_STREAM("No white detected!");
        drive_robot(0,0);
   }  
   

}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"process_image");

    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw",10,process_image_callback);

    ros::spin();

    return 0;
}
