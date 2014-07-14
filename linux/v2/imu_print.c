#include "imu.h"

int main(int argc, char** argv) {
    imu_data heading;
    
    imuConfig();
    
    while(1) {
        imuGetData(&heading);
        printf("RPY: %.2f, %.2f, %.2f\n\n", heading.rpy[0], heading.rpy[1], heading.rpy[2]);
    }
}
