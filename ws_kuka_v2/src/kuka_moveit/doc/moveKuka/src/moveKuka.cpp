//Header file moveKuka.h consist of all the necessary functions
#include "moveKuka.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveKuka");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Creat a object of class moveKuka
    moveKuka obj_moveKuka;
    //Start the simulation
    obj_moveKuka.Start();

    return 0;
}
