#include "imu.h"
#include "lsm303dlhc.h"
#include "tick.h"
#include "robot.h"
#include "controller.h"
#include "freeIMU.h"
#include "stdio.h"
#include "motors.h"

int main(int argc, char** argv) {
    
    int loop_timer;
    imu_data heading;
    motor_set message;
    
    state_vector state_x, state_y;
    float force_x, force_y;
    float mass = 1;
    
    int32_t mot_vel[3];                         // motor velocity registers (encoder ticks/sec)
    float ball_vel[3];                          // x, y, and theta velocities of ball
    
    mot_vel[0] = 0;
    mot_vel[1] = 0;
    mot_vel[2] = 0;
    
    imuConfig();
    loop_timer = makeTimer(LOOP_FREQ);
    
    while(1) {
        /*sensorRead(&heading);          // read 9-dof sensor
        AHRSupdate(&heading);          // call AHRS update routine
        getYawPitchRoll(&heading);     // return RPY representation
        
        state_x.angle = heading.rpy[0];
        state_x.angle_dot = heading.gyro[0] * 180 / PI;
        state_x.pos += (ball_vel[0] * LOOP_PERIOD);
        state_x.pos_dot = ball_vel[0];
        
        state_y.angle = heading.rpy[1];
        state_y.angle_dot = heading.gyro[1] * 180 / PI;
        state_y.pos += (ball_vel[1] * LOOP_PERIOD);
        state_y.pos_dot = ball_vel[1];
        
        force_x = gain.k[0]*state_x.angle + gain.k[1]*state_x.angle_dot + gain.k[2]*state_x.pos + gain.k[3]*state_x.pos_dot;
        force_y = gain.k[0]*state_y.angle + gain.k[1]*state_y.angle_dot + gain.k[2]*state_y.pos + gain.k[3]*state_y.pos_dot;
        
        ball_vel[0] += (force_x / mass) * LOOP_PERIOD;
        ball_vel[1] += (force_y / mass) * LOOP_PERIOD;
        ball_vel[2] = 0;
        
        printf("heading.rpy  =   %f \n",heading.rpy[0]);
        
        ballIK(ball_vel, mot_vel);       // calculate motor velocities from IK function
        */ 
        
        message.motor = 1.0;
        message.velocity = 100;
        
        motorVelocitySet(&message);
        
        waitOnTimer(loop_timer);         // wait for next timer tick
    }
}