#ifndef VISARM_H
#define VISARM_H

#include <string>
#include <vector>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpRxyzVector.h>

class VisArm {
private:
  int serial_fd;
  std::string serial_port;
  std::vector<double> current_angles;
  
  double ll[6];
  double initial_offset[3];
  
  std::string sendCommand(const std::string &cmd);
  std::vector<std::string> findSerialPorts();
  vpHomogeneousMatrix dhTransform(double a, double alpha, double d, double theta);
  bool openSerialPort(const std::string &port);

public:
  VisArm();
  ~VisArm();
  
  bool connect(const std::string &port = "");
  std::vector<double> getJointAngles();
  vpHomogeneousMatrix fkine(const std::vector<double> &q);
  vpHomogeneousMatrix get_eMc();
  vpHomogeneousMatrix getBaseToCamera();
  vpPoseVector getPoseVector();
  void disconnect();
};

#endif // VISARM_H