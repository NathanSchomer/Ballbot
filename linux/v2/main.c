#include "imu.h"
#include "lsm303dlhc.h"

int main(int argc, char** argv) {
    imu_data heading;
    imu_data pimu_data;
    
    imuConfig();
    
    while(1) {
        sensorRead(&pimu_data);          // read 9-dof sensor
    }
}