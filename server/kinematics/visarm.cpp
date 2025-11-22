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
#include <cctype>

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

std::string trim(const std::string &s)
{
    size_t a = s.find_first_not_of(" \r\n\t");
    size_t b = s.find_last_not_of(" \r\n\t");
    if (a == std::string::npos) return "";
    return s.substr(a, b - a + 1);
}

std::string VisArm::sendCommand(const std::string &cmd)
{
    if (serial_fd < 0) {
        std::cerr << "[VisArm] Serial not connected\n";
        return "";
    }

    tcflush(serial_fd, TCIFLUSH);

    // Send command
    std::string msg = cmd;
    if (msg.back() != '\n') msg.push_back('\n');
    write(serial_fd, msg.c_str(), msg.size());
    tcdrain(serial_fd);

    bool is_get = cmd.rfind("GET", 0) == 0;
    bool is_set = cmd.rfind("SET", 0) == 0;
    int timeout = is_set ? 10000 : 2000;

    while (true)
    {
        std::string raw = readLine(timeout);
        std::string line = trim(raw);

        // std::cout << "[VisArm] << " << line << "\n";

        // Success for SET
        if (line == "READY" || line == "OK")
            return line;

        if (line.rfind("ANGLES", 0) == 0)
            return line;

        if (line.rfind("ERR", 0) == 0)
            return line;
    }
}

std::vector<double> VisArm::parseAngles(const std::string &line)
{
    std::vector<double> a;
    std::istringstream iss(line);
    std::string tag;
    iss >> tag;   // "ANGLES"

    double v;
    while (iss >> v)
        a.push_back(v);

    return a;
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
std::vector<double> VisArm::getJointAngles()
{
    std::string resp = sendCommand("GET");
    if (resp.rfind("ANGLES", 0) != 0) {
        std::cerr << "[VisArm] Invalid GET response: " << resp << "\n";
        return current_angles;
    }

    auto a = parseAngles(resp);
    if (a.size() >= 5)
        current_angles = {a[0], a[1], a[2], a[3], a[4]};

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
    
    vpRxyzVector rxyz(0, 0, -M_PI);
    vpRotationMatrix R(rxyz);
    
    return vpHomogeneousMatrix(t, R);
  }


vpHomogeneousMatrix VisArm::get_rMe(){
    std::vector<double> q = getJointAngles();
    std::cout << "[VISARM] Joint angles for FK: ";
    for (size_t i = 0; i < q.size(); i++) {
      std::cout << q[i] << " ";
    }
    std::cout << std::endl;
    
    return fkine(q);
}

// -------------------------------------------------------------------------
// 5. Combined base → camera
// -------------------------------------------------------------------------
vpHomogeneousMatrix VisArm::getBaseToCamera() {
    std::vector<double> q = getJointAngles();
    std::cout << "[VISARM] Joint angles for FK: ";
    for (size_t i = 0; i < q.size(); i++) {
      std::cout << q[i] << " ";
    }
    std::cout << std::endl;
    
    return fkine(q) * get_eMc();
  }

// -------------------------------------------------------------------------
// 6. Position in vpPoseVector (for saving YAML)
// -------------------------------------------------------------------------
vpPoseVector VisArm::getPoseVector() {
    vpHomogeneousMatrix M = get_rMe();
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
  if (serial_fd < 0) {
    std::cerr << "[VisArm] Serial port not connected" << std::endl;
    return false;
  }

  // Mimic Python behavior:
  // cmd = "SET " + " ".join(str(int(a)) for a in angles) + " 0\n"
  // ser.write(cmd); line = ser.readline()

  // Flush any pending input
  tcflush(serial_fd, TCIFLUSH);

  std::ostringstream cmd;
  cmd << "SET ";
  for (size_t i = 0; i < 6; ++i) {
    int ai = static_cast<int>(std::lround(angles[i]));
    cmd << ai << " ";
  }
  cmd << 0 << "\n"; // trailing 0 like Python code

  std::string cmd_str = cmd.str();
  ssize_t w = write(serial_fd, cmd_str.c_str(), cmd_str.size());
  (void)w;
  tcdrain(serial_fd);

  // Read lines up to a generous timeout (servo motions can be long)
  const auto timeout = std::chrono::seconds(30);
  auto start = std::chrono::steady_clock::now();
  std::string line;
  std::string full_log;
  char ch;
  while (std::chrono::steady_clock::now() - start < timeout) {
    int n = read(serial_fd, &ch, 1);
    if (n == 1) {
      if (ch == '\r') continue; // ignore CR
      if (ch == '\n') {
        if (!line.empty()) {
          full_log += line + "\n";
          // Debug echo
          // std::cout << "[VisArm] Line: '" << line << "'" << std::endl;
          // Status checks
          if (line.find("ERR") != std::string::npos) {
            std::cerr << "[VisArm] Error reported." << std::endl;
            return false;
          }
          if (line.find("READY") != std::string::npos || line.find("OK") != std::string::npos) {
            std::cout << "[VisArm] Motion complete (" << (line.find("READY") != std::string::npos ? "READY" : "OK") << ")." << std::endl;
            return true;
          }
          // Ignore RAW and COUNT lines; continue accumulating
          line.clear();
        }
        continue;
      }
      line.push_back(ch);
      // Safeguard against extremely long lines
      if (line.size() > 512) {
        full_log += line + "\n";
        line.clear();
      }
    } else {
      // No data yet; brief sleep
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  std::cerr << "[VisArm] Timeout waiting for READY. Collected output:\n" << full_log << std::endl;
  return false;
}

bool VisArm::setHome(){
    std::vector<double> home_angles = {0, 0, 0, 0, 0, 0};
    return setJointAngles(home_angles);
  }

std::string VisArm::readLine(int timeout_ms)
{
    std::string line;
    char c;

    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start <
           std::chrono::milliseconds(timeout_ms))
    {
        int n = read(serial_fd, &c, 1);
        if (n == 1)
        {
            if (c == '\r') continue;
            if (c == '\n') return line;
            line.push_back(c);
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    return "";  // timeout
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