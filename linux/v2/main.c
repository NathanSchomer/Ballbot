#include "imu.h"
#include "lsm303dlhc.h"

int main(int argc, char** argv) {
    imu_data heading;
    
    imuConfig();
    
    while(1) {
        sensorRead();          // read 9-dof sensor
    }
}

