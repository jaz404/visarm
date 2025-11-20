#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <kinematics/visarm.h>

int main(){
    VisArm robot;
    if (!robot.connect()) {
        std::cerr << "Failed to connect to VisArm." << std::endl;
        return -1;
    }

    std::vector<double> joint_angles = robot.getJointAngles();
    std::cout << "Current Joint Angles: ";
    for (const auto &angle : joint_angles) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    vpHomogeneousMatrix eMc = robot.get_eMc();
    std::cout << "End-Effector to Camera Transformation:\n" << eMc << std::endl;

    vpHomogeneousMatrix baseToCamera = robot.getBaseToCamera();
    std::cout << "Base to Camera Transformation:\n" << baseToCamera << std::endl;

    sleep(2);

    // Move to angles [0, 45, 30, 0, 0]
    std::vector<double> target_angles = {0, 45, 45, 45, 0, 0};
    if (robot.setJointAngles(target_angles)) {
        std::cout << "Robot moved to target angles successfully." << std::endl;
    } else {
        std::cerr << "Failed to move robot to target angles." << std::endl;
    }

    sleep(2);
    robot.setHome();
    std::cout << "Robot returned to home position." << std::endl;

    sleep(2);
    robot.disconnect();
    return 0;
}