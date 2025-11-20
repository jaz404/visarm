#include <iostream>
#include <sstream>
#include <vector>
#include <visp3/core/vpException.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpXmlParserCamera.h>

#include <visp3/io/vpImageIo.h>

#include "kinematics/visarm.h"


#if defined(VISP_HAVE_REALSENSE2) && \
    defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_PUGIXML) && \
    defined(VISP_HAVE_MODULE_GUI) && defined(VISP_HAVE_MODULE_SENSOR)


// ===========================================================================
//                           USAGE MESSAGE
// ===========================================================================
void usage(const char **argv, int error)
{
  std::cout << "Usage: " << argv[0]
            << " [--serial <port>] [--output-folder <path>]\n";
  std::cout << "\nOptions:\n";
  std::cout << "  --serial <port>        Serial port (e.g., /dev/ttyACM0, /dev/ttyUSB0)\n";
  std::cout << "                         If not specified, will auto-detect Arduino\n";
  std::cout << "  --output-folder <path> Output folder for images and poses (default: ./)\n";
  std::cout << "\nExample:\n";
  std::cout << "  " << argv[0] << " --serial /dev/ttyACM0\n";
  std::cout << "  " << argv[0] << " (auto-detect)\n";
  if (error)
    std::cout << "\nError: unknown argument: " << argv[error] << "\n";
}


// ===========================================================================
//                                   MAIN
// ===========================================================================
int main(int argc, const char **argv)
{
  try {
    std::string opt_serial_port = "";
    std::string opt_output_folder = "./";

    // Parse arguments
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--serial" && i + 1 < argc)
        opt_serial_port = argv[++i];
      else if (std::string(argv[i]) == "--output-folder" && i + 1 < argc)
        opt_output_folder = argv[++i];
      else {
        usage(argv, i);
        return EXIT_FAILURE;
      }
    }

    // Create output folder
    if (!vpIoTools::checkDirectory(opt_output_folder))
      vpIoTools::makeDirectory(opt_output_folder);

    vpImage<unsigned char> I;

    // -----------------------------------------------------------------------
    // VISARM ROBOT
    // -----------------------------------------------------------------------
    VisArm robot;
    if (!robot.connect(opt_serial_port)) {
      std::cerr << "Failed to connect to Arduino." << std::endl;
      std::cerr << "Make sure:" << std::endl;
      std::cerr << "  1. Arduino is connected via USB" << std::endl;
      std::cerr << "  2. Arduino is running client_code.ino" << std::endl;
      std::cerr << "  3. You have permission to access serial port (add user to dialout group)" << std::endl;
      return EXIT_FAILURE;
    }
    
    // Test connection by getting joint angles
    std::vector<double> test_angles = robot.getJointAngles();
    std::cout << "Successfully connected! Current joint angles retrieved." << std::endl;
    std::cout << "Joint angles: ";
    for (size_t i = 0; i < test_angles.size(); i++) {
      std::cout << test_angles[i] << " ";
    }
    std::cout << std::endl;

    // -----------------------------------------------------------------------
    // REALSENSE CAMERA
    // -----------------------------------------------------------------------
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);

    g.open(config);
    g.acquire(I);

    unsigned int width = I.getWidth();
    unsigned int height = I.getHeight();
    std::cout << "Image size: " << width << " x " << height << "\n";

    // Save intrinsics
    vpCameraParameters cam =
        g.getCameraParameters(RS2_STREAM_COLOR,
                              vpCameraParameters::perspectiveProjWithDistortion);

    vpXmlParserCamera xml;
    xml.save(cam, opt_output_folder + "/custom_camera.xml", "Camera",
             width, height);

    // Display
    #if defined(VISP_HAVE_X11)
        vpDisplayX d(I, 10, 10, "Color image");
    #elif defined(VISP_HAVE_OPENCV)
        vpDisplayOpenCV d(I, 10, 10, "Color image");
    #else
        std::cerr << "No display available" << std::endl;
    #endif

    unsigned cpt = 0;
    bool end = false;

    // Poses to go to
    std::vector<std::vector<double>> poses = {
        {0, 0, 0, 0, 0, 0},
        {30, 0, 0, 0, 0, 0},
        {0, 30, 0, 0, 0, 0},
        {0, 0, 30, 0, 0, 0},
        {0, 0, 0, 30, 0, 0},
        {0, 0, 0, 0, 30, 0},
        {0, 0, 0, 0, 0, 30},
        {45, 30, -30, 15, -15, 10}
    };

    // -----------------------------------------------------------------------
    // MAIN LOOP
    // -----------------------------------------------------------------------
    // while (!end) {
    //   g.acquire(I);
    //   vpDisplay::display(I);

    //   vpDisplay::displayText(I, 15, 15,
    //                          "Left-click = save image + pose\n"
    //                          "Right-click = quit",
    //                          vpColor::red);

    //   vpMouseButton::vpMouseButtonType button;
    //   if (vpDisplay::getClick(I, button, false)) {
    //     if (button == vpMouseButton::button1) {
    //       cpt++;

    //       vpPoseVector pose = robot.getPoseVector();

    //       std::stringstream img_name, pose_name;
    //       img_name << opt_output_folder << "/image_" << cpt << ".png";
    //       pose_name << opt_output_folder << "/pose_" << cpt << ".yaml";

    //       std::cout << "Saving " << img_name.str()
    //                 << " + " << pose_name.str() << "\n";

    //       vpImageIo::write(I, img_name.str());
    //       pose.saveYAML(pose_name.str(), pose);
    //     }
    //     if (button == vpMouseButton::button3)
    //       end = true;
    //   }
    //   vpDisplay::flush(I);
    // }
    for (size_t i = 0; i < poses.size(); i++) {
      std::cout << "Moving to pose " << i + 1 << " / " << poses.size() << "\n";
      if (!robot.setJointAngles(poses[i])) {
        std::cerr << "Failed to move to pose\n";
        continue;
      }

      // Wait a bit for the robot to reach the position
      vpTime::wait(2000);

      // Acquire image
      g.acquire(I);
      vpDisplay::display(I);

      vpPoseVector pose = robot.getPoseVector();

      std::stringstream img_name, pose_name;
      img_name << opt_output_folder << "/image_" << i + 1 << ".png";
      pose_name << opt_output_folder << "/pose_" << i + 1 << ".yaml";

      std::cout << "Saving " << img_name.str()
                << " + " << pose_name.str() << "\n";

      vpImageIo::write(I, img_name.str());
      pose.saveYAML(pose_name.str(), pose);

      vpDisplay::flush(I);
    }

  } catch (const vpException &e) {
    std::cerr << "ViSP exception: " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}

#else
int main() {
  std::cout << "ViSP missing required modules.\n";
  return 0;
}
#endif
