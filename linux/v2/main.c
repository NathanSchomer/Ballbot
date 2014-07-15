#include "imu.h"
#include "lsm303dlhc.h"
#include "tick.h"
#include "robot.h"

int main(int argc, char** argv) {
    
    int loop_timer;
    imu_data pimu_data;
    
    imuConfig();
    loop_timer = makeTimer(LOOP_FREQ);
    
    while(1) {
        sensorRead(&pimu_data);          // read 9-dof sensor   
        waitOnTimer(loop_timer);         // wait for next timer tick
    }
}