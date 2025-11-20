#include "visarm.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <dirent.h>
#include <thread>
#include <chrono>

// Constructor
VisArm::VisArm() : serial_fd(-1), current_angles(5, 0.0) {
  ll[0] = 2.0;
  ll[1] = 10.3;
  ll[2] = 9.6;
  ll[3] = 4.0;
  ll[4] = 2.5;
  ll[5] = 5.0;
  
  initial_offset[0] = 0;
  initial_offset[1] = 0;
  initial_offset[2] = 9.5;
}

// Destructor
VisArm::~VisArm() {
  disconnect();
}

std::string VisArm::sendCommand(const std::string &cmd) {
    if (serial_fd < 0) {
      std::cerr << "[VisArm] Serial port not connected" << std::endl;
      return "";
    }
    
    std::string full_cmd = cmd + "\n";
    write(serial_fd, full_cmd.c_str(), full_cmd.length());
    
    // Wait for response
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    char buffer[1024] = {0};
    int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
      buffer[bytes_read] = '\0';
      return std::string(buffer);
    }
    return "";
  }

std::vector<std::string> VisArm::findSerialPorts() {
    std::vector<std::string> ports;
    DIR *dir = opendir("/dev");
    if (dir) {
      struct dirent *entry;
      while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name.find("ttyACM") == 0 || name.find("ttyUSB") == 0) {
          ports.push_back("/dev/" + name);
        }
      }
      closedir(dir);
    }
    return ports;
  }

vpHomogeneousMatrix VisArm::dhTransform(double a, double alpha, double d, double theta) {
    double alpha_rad = alpha * M_PI / 180.0;
    double theta_rad = theta;
    
    vpRotationMatrix R;
    R[0][0] = cos(theta_rad);
    R[0][1] = -sin(theta_rad) * cos(alpha_rad);
    R[0][2] = sin(theta_rad) * sin(alpha_rad);
    R[1][0] = sin(theta_rad);
    R[1][1] = cos(theta_rad) * cos(alpha_rad);
    R[1][2] = -cos(theta_rad) * sin(alpha_rad);
    R[2][0] = 0;
    R[2][1] = sin(alpha_rad);
    R[2][2] = cos(alpha_rad);
    
    vpTranslationVector t;
    t[0] = a * cos(theta_rad);
    t[1] = a * sin(theta_rad);
    t[2] = d;
    
    return vpHomogeneousMatrix(t, R);
  }

bool VisArm::connect(const std::string &port) {
    if (!port.empty()) {
      serial_port = port;
    } else {
      // Auto-detect Arduino
      std::cout << "[VisArm] Auto-detecting Arduino..." << std::endl;
      std::vector<std::string> ports = findSerialPorts();
      
      if (ports.empty()) {
        std::cerr << "[VisArm] No serial ports found" << std::endl;
        return false;
      }
      
      // Try each port
      for (const auto &p : ports) {
        std::cout << "[VisArm] Trying " << p << "..." << std::endl;
        if (openSerialPort(p)) {
          // Check for READY message
          std::this_thread::sleep_for(std::chrono::seconds(2));
          
          char buffer[256] = {0};
          int bytes = read(serial_fd, buffer, sizeof(buffer) - 1);
          if (bytes > 0) {
            std::string response(buffer);
            if (response.find("READY") != std::string::npos) {
              std::cout << "[VisArm] Found Arduino on " << p << std::endl;
              serial_port = p;
              return true;
            }
          }
          
          // Send GET command to test
          std::string test_response = sendCommand("GET");
          if (test_response.find("ANGLES") != std::string::npos) {
            std::cout << "[VisArm] Found Arduino on " << p << std::endl;
            serial_port = p;
            return true;
          }
          
          close(serial_fd);
          serial_fd = -1;
        }
      }
      
      std::cerr << "[VisArm] Could not find Arduino on any port" << std::endl;
      return false;
    }
    
    return openSerialPort(serial_port);
  }

bool VisArm::openSerialPort(const std::string &port) {
    serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
      std::cerr << "[VisArm] Error opening " << port << ": " << strerror(errno) << std::endl;
      return false;
    }
    
    // Configure serial port
    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
      std::cerr << "[VisArm] Error getting serial attributes" << std::endl;
      close(serial_fd);
      serial_fd = -1;
      return false;
    }
    
    // Set baud rate to 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    // 8N1 mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
      std::cerr << "[VisArm] Error setting serial attributes" << std::endl;
      close(serial_fd);
      serial_fd = -1;
      return false;
    }
    
    std::cout << "[VisArm] Connected to " << port << " at 115200 baud" << std::endl;
    
    // Wait for Arduino to reset
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Flush any initial data
    tcflush(serial_fd, TCIOFLUSH);
    
    return true;
  }

// -------------------------------------------------------------------------
// 2. Read the 5 joint angles from the robot (we use 5 joints, not counting gripper)
// -------------------------------------------------------------------------
std::vector<double> VisArm::getJointAngles() {
    std::string response = sendCommand("GET");
    
    if (response.empty()) {
      std::cerr << "[VisArm] No response from GET command" << std::endl;
      return current_angles;
    }
    
    std::istringstream iss(response);
    std::string prefix;
    iss >> prefix;
    
    if (prefix.find("ANGLES") != std::string::npos) {
      std::vector<double> angles;
      double angle;
      int count = 0;
      
      while (iss >> angle && count < 7) {
        angles.push_back(angle);
        count++;
      }
      
      if (count >= 5) {
        current_angles = std::vector<double>(angles.begin(), angles.begin() + 5);
        std::cout << "[VisArm] Joint angles: ";
        for (size_t i = 0; i < current_angles.size(); i++) {
          std::cout << current_angles[i] << " ";
        }
        std::cout << std::endl;
      } else {
        std::cerr << "[VisArm] Not enough angles received: " << count << std::endl;
      }
    }
    
    return current_angles;
  }

// -------------------------------------------------------------------------
// 3. Forward kinematics from base → end-effector
// DH Parameters from Python code:
// Joint 1: {'a': 0, 'alpha': 90, 'd': ll[0], 'theta': θ1}
// Joint 2: {'a': ll[1], 'alpha': 0, 'd': 0, 'theta': θ2 + π/2}
// Joint 3: {'a': ll[2], 'alpha': 0, 'd': 0, 'theta': -θ3}
// Joint 4: {'a': ll[4], 'alpha': 90, 'd': 0, 'theta': -θ4 + π/2}
// Joint 5: {'a': 0, 'alpha': 0, 'd': ll[3] + ll[5], 'theta': θ5}
// -------------------------------------------------------------------------
vpHomogeneousMatrix VisArm::fkine(const std::vector<double> &q) {
    if (q.size() < 5) {
      std::cerr << "[VisArm] fkine requires 5 joint angles" << std::endl;
      return vpHomogeneousMatrix();
    }
    
    std::vector<double> q_rad(5);
    for (size_t i = 0; i < 5; i++) {
      q_rad[i] = q[i] * M_PI / 180.0;
    }
    
    vpTranslationVector t0(initial_offset[0], initial_offset[1], initial_offset[2]);
    vpHomogeneousMatrix T = vpHomogeneousMatrix(t0, vpRotationMatrix());
    
    T = T * dhTransform(0, 90, ll[0], q_rad[0]);
    T = T * dhTransform(ll[1], 0, 0, q_rad[1] + M_PI/2);
    T = T * dhTransform(ll[2], 0, 0, -q_rad[2]);
    T = T * dhTransform(ll[4], 90, 0, -q_rad[3] + M_PI/2);
    T = T * dhTransform(0, 0, ll[3] + ll[5], q_rad[4]);
    
    return T;
  }

// -------------------------------------------------------------------------
// 4. End-effector → camera transform
// This will be calibrated by this program, but we provide initial estimate
// -------------------------------------------------------------------------
vpHomogeneousMatrix VisArm::get_eMc() {
    vpTranslationVector t(10.0, 0, 0);
    
    vpRxyzVector rxyz(-M_PI/2, 0, 0);
    vpRotationMatrix R(rxyz);
    
    return vpHomogeneousMatrix(t, R);
  }

// -------------------------------------------------------------------------
// 5. Combined base → camera
// -------------------------------------------------------------------------
vpHomogeneousMatrix VisArm::getBaseToCamera() {
    std::vector<double> q = getJointAngles();
    return fkine(q) * get_eMc();
  }

// -------------------------------------------------------------------------
// 6. Position in vpPoseVector (for saving YAML)
// -------------------------------------------------------------------------
vpPoseVector VisArm::getPoseVector() {
    vpHomogeneousMatrix M = getBaseToCamera();
    return vpPoseVector(M);
  }

// -------------------------------------------------------------------------
// 7. Set joint angles on the robot
// -------------------------------------------------------------------------
bool VisArm::setJointAngles(const std::vector<double> &angles) {
    if (angles.size() < 6) {
      std::cerr << "[VisArm] setJointAngles requires 6 angles" << std::endl;
      return false;
    }
    
    std::ostringstream oss;
    oss << "SET ";
    for (size_t i = 0; i < 6; i++) {
      oss << angles[i] << " ";
    }
    
    std::string response = sendCommand(oss.str());
    if (response.find("OK") != std::string::npos) {
      std::cout << "[VisArm] Joint angles set successfully" << std::endl;
      return true;
    } else {
      std::cerr << "[VisArm] Failed to set joint angles" << std::endl;
      std::cout << "[VisArm] Response: " << response << std::endl;
      return false;
    }
  }

bool VisArm::setHome(){
    std::vector<double> home_angles = {0, 0, 0, 0, 0, 0};
    return setJointAngles(home_angles);
  }

// -------------------------------------------------------------------------
// 8. Disconnect from robot
// -------------------------------------------------------------------------
void VisArm::disconnect() {
    if (serial_fd >= 0) {
      close(serial_fd);
      serial_fd = -1;
      std::cout << "[VisArm] Disconnected from Arduino" << std::endl;
    }
  }