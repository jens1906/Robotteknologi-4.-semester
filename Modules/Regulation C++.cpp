#include <iostream>
#include <cmath>
#include <array>

using namespace std;

array<double, 3> desPos; // desired position from GUI
array<double, 3> curPos; // current position from Vicon

class PID { // PID controller class
    public:
        double kp, ki, kd;
        double preError, error, integral, derivatve, output;

        PID(array<double, 3> parameters) {
            kp = parameters[0];
            ki = parameters[1];
            kd = parameters[2];
            preError = 0;
        }

        double calc(double setPoint, double curVal) { // Calculates PID output
            error = setPoint - curVal;
            integral += error;
            derivative = error - preError;
            output = kp * error + ki * integral + kd * derivative;
            preError = error;
            return output;
        }
        
    private:
        float error, integral, derivative;
};

class Drone { // Drone class - contains all PID controllers
    public:
        array<double, 3> roll, pitch, thrust;

        Drone(array<double, 3> roll, array<double, 3> pitch, array<double, 3> thrust) : pidRoll(roll), pidPitch(pitch), pidThrust(thrust) {
            roll = roll;
            pitch = pitch;
            thrust = thrust;
        }

        double calc(array<double, 3> desPos, array<double, 3> curPos) { // Calculates PID output from all controllers
            double rollOutput = pidRoll.calc(desPos[0], curPos[0]);
            double pitchOutput = pidPitch.calc(desPos[1], curPos[1]);
            double thrustOutput = pidThrust.calc(desPos[2], curPos[2]);
        return rollOutput, pitchOutput, thrustOutput;
        }

    private:
        PID pidRoll, pidPitch, pidThrust;
    
};

main( ){
    array<double, 3> rollParameter = {1, 1, 1};
    array<double, 3> pitchParameter = {1, 1, 1};
    array<double, 3> thrustParameter = {1, 1, 1};

    desPos = {1, 1, 1};
    curPos = {0, 0, 0};

    Drone drone(rollParameter, pitchParameter, thrustParameter);
    drone.calc(desPos, curPos);

}

