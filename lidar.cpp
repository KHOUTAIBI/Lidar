#include "CYdLidar.h"
#include <string>
// #include <cmath>
using namespace std;
using namespace ydlidar;

int plot();

int main(int argc, char *argv[]) {

    // init system signal
  ydlidar::os_init();

  
  CYdLidar laser;
  
  std::string port = "/dev/ttyUSB0";
  
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

  
  /// lidar baudrate
  int optval = 128000;
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));


  // Set The degrees of the Lidar, maximum and minimum
  float angle_values = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &angle_values, sizeof(float));
  angle_values = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &angle_values, sizeof(float));
  
  //     Set the maximum and minimum range of the lidar
  angle_values = 16.f;
  laser.setlidaropt(LidarPropMaxRange, &angle_values, sizeof(float));
  angle_values = 0.1f;
  laser.setlidaropt(LidarPropMinRange, &angle_values, sizeof(float));

// initialize SDK and YDLidar
  bool response = laser.initialize();

// If response 
  if (response) {   

    //Start the device scanning routine which runs on a separate thread and enable motor.
    response = laser.turnOn();

// Otherwise, we stop and describe error
  } else {
    fprintf(stderr, "%s\n", laser.DescribeError());
    fflush(stderr);
  }
  
  // Turn On sucscess and loop  
  while (response && ydlidar::os_isOk()) {
    
    // Start laser scan
    LaserScan scan;

    // Trying simple process
    if (laser.doProcessSimple(scan)) {
    
    //Getting the size of points and the points array, seems to be 500 ?
        std::vector<LaserPoint> points = scan.points;
    
    for (int point = 0; point < points.size() ; point++) {
                // Calculate the angle of the current point

                float angle = points[point].angle ;
                // Range value for the current point

                float range = points[point].range;
                // Print the angle and range

                printf("Point number %u with: Angle = %.5f°, Range = %.5f m\n", point, angle, range);

                // Flushing the data ie forcing an update
                fflush(stdout);
        }   
    } 
    
    // If error, flush and print error
    else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }

  }

  // Stop the device scanning thread and disable motor.
  
  laser.turnOff();
  
  // Uninitialize the SDK and Disconnect the LiDAR.
  laser.disconnecting();
  
  return 0;
}
