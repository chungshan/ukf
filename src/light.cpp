#include <ros/ros.h>
#include <vector>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  std::vector<int> light;

  light.assign(12,0);

  int i = 0;
  light[0] = 1;
  for (i = 0 ; i < 12 ; i++){
    light[i] = 1;
    light[i - 1] = 0;

    std::cout << "----light " << i+1 << " on----" << std::endl;
    for (int j = 0 ; j < 12 ; j ++){

      std::cout << light[j] << std::endl;

    }

  }



}
