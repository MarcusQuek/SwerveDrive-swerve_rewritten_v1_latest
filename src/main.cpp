#include "main.h"

void disabled(){}
void competition_initialize(){}

//Function to determine sign of a integer variable, returns bool
template <typename T> int sgn(T val){
    return (T(0) < val) - (val < T(0));
}



/*
void serialRead(void* params){
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    pros::delay(10);
    pros::screen::set_pen(COLOR_BLUE);
    double distX, distY = 0;
    while(true){
        uint8_t buffer[256];
        int bufLength = 256;
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, bufLength);
        if(nRead >= 0){
            std::stringstream dataStream("");
            bool recordOpticalX, recordOpticalY = false;
            for(int i = 0;i < nRead; i++){
                char thisDigit = (char)buffer[i];
                if(thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X'||  thisDigit == 'C'||  thisDigit == 'Y'){
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                if(thisDigit == 'C'){
                    recordOpticalX = false;
                    dataStream >> distX;
                    pros::lcd::print(1, "Optical Flow:");
                    pros::lcd::print(2, "distX: %.2lf", distX/100);
                    dataStream.str(std::string());
                }
                if(thisDigit == 'D'){
                    recordOpticalY = false;
                    dataStream >> distY;
                    global_distY = distY/100;
                    pros::lcd::print(3, "distY: %.2lf", distY/100);
                    dataStream.str(std::string());
                }
                if (recordOpticalX) dataStream << (char)buffer[i];
                if (recordOpticalY) dataStream << (char)buffer[i];
                if (thisDigit == 'X') recordOpticalX = true;
                if (thisDigit == 'Y') recordOpticalY = true;
            }
        }
        pros::Task::delay(25);
    }
}*/

void brake(){ //brakes all base motors
    luA.brake();
    ruA.brake();
    luB.brake();
    ruB.brake();
    llA.brake();
    rlA.brake();
    llB.brake();
    rlB.brake();
    pros::delay(1);
}

void tareBaseMotorEncoderPositions() //tares all base motor encoder positions
{
    luA.tare_position();
    ruA.tare_position();
    luB.tare_position();
    ruB.tare_position();
    llA.tare_position();
    rlA.tare_position();
    llB.tare_position();
    rlB.tare_position();
    pros::delay(1);
}

void clampVoltage(double lu, double ll, double ru, double rl)
{
    //if any of lu, ll, ru or rl are too big, we need to scale them, and we must scale them all by the same amount so we dont throw off the proportions
    if(fabs(lu) > MAX_VOLTAGE || fabs(ll) > MAX_VOLTAGE || fabs(ru) > MAX_VOLTAGE || fabs(rl) > MAX_VOLTAGE)
    {
        //figure out which of lu, ll, ru or rl has the largest magnitude
        double max = fabs(lu);
        if(max < fabs(ll))
            max = fabs(ll);
        if(max < fabs(ru))
            max = fabs(ru);
        if(max < fabs(rl))
            max = fabs(rl);
        double VoltageScalingFactor = max / MAX_VOLTAGE; //this will definitely be positive, hence it wont change the sign of lu, ll, ru or rl.
        lu = lu / VoltageScalingFactor;
        ll = ll / VoltageScalingFactor;
        ru = ru / VoltageScalingFactor;
        rl = rl / VoltageScalingFactor;
    }
}
double wrapAngle(double angle){ //forces the angle to be within the -180 < angle < 180 range
    if (angle > 180.0)
        while (angle > 180.0)
            angle -= 360.0;
    else if (angle < -180.0)
        while (angle< -180.0)
            angle += 360.0;  
    return angle;
}

double getNormalizedSensorAngle(pros::Rotation &sensor){ //Converts rotational sensor readings into degrees and bounds it between -180 to 180
    double angle = sensor.get_angle() / 100.0; //Convert from centidegrees to degrees
    return wrapAngle(angle); //forces the angle to be within the -180 < angle < 180 range
}

vector3D normalizeJoystick(int x_in, int y_in){ //convert translation joystick input to polar vector
    double angle = atan2(y_in, x_in) * TO_DEGREES; //angle of translation polar vector. Note that atan2 automatically considers the sign to deal with the trigonometry quadrant
    double scaleLength;
    double magnitude; //magnitude of translation polar vector
    double length = sqrt(x_in * x_in + y_in * y_in);
    vector3D out;
    if(length < DEADBAND){ //if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); //assign zero values to the xyz attributes of the vector3D named "out"
        return out;
    }
    //forcing the joystick output to be a circle not the square bounding box of the joystick
    //for any radial line of the circle, we find its length from the deadband radius to the radius of the circle of the joystick, then map the speed from 0 to 1 of that length
    //this means that in any direction, we can reach our full range of speeds without being limited
    //use CSC or SEC as required
    if((angle > 45.0  && angle < 135.0) || (angle < -45.0 && angle > -135.0))
        scaleLength = 127.0 / sin(angle * TO_RADIANS);
    else
        scaleLength = 127.0 / cos(angle * TO_RADIANS);
    
    scaleLength = fabs(scaleLength) - DEADBAND; //force scaleLength to be positive and subtract deadband
    magnitude = (length - DEADBAND) / scaleLength; //find magnitude of translation vector and scale it down (note that magnitude will always be positive)
    
    out.load(magnitude * cos(angle * TO_RADIANS), magnitude * sin(angle * TO_RADIANS), 0.0); //assign values to the xyz attributes of the vector3D named "out"
    return out;
}

vector3D normalizeRotation(int x_in){ //get rotation speed from rotation joystick
    vector3D out;
    double scaleLength = 127.0 - DEADBAND;
    if(abs(x_in) < DEADBAND){ //if the joystick is too close to the origin, dont bother moving (this is to correct for stick drift, where the joystick doesnt default to the 0,0 position due to physical damage)
        out.load(0.0, 0.0, 0.0); //assign values to the xyz attributes of the vector3D named "out"
        return out;
    }
    double value = (abs(x_in) - DEADBAND) / scaleLength; //find magnitude of rotation and scale it down
    if(x_in < 0){
        value = value * -1.0;
    }
    //this is SUPPOSED to be a vector, its not wrong
    //both normaliseRotation and normaliseJoystick return a vector for standardisation. This is intended behaviour.
    out.load(0.0, 0.0, value); //assign values to the xyz attributes of the vector3D named "out"
    return -out;
}


double angle(vector3D v1, vector3D v2){
    double dot = v1 * v2;
    double det = v1.x * v2.y - v1.y * v2.x;
    return -atan2(det, dot); //atan2 automatically considers the sign to deal with the trigonometry quadrant
}

double max(double a, double b) { //returns the larger of two doubles
    return (a > b)? a : b;
}

double min(double a, double b) { //returns the smaller of two doubles
    return (a < b)? a : b;
}

void moveBase(){
    double v_right_velocity;
    double v_left_velocity;
    double left_angle;
    double right_angle;
    double left_target_angle;
    double right_target_angle;
    vector3D rotational_v_vector;
    
    vector3D current_left_vector;
    vector3D current_right_vector;

    double l_error = 0.0;
    double r_error = 0.0;

    double current_l_velocity = 0.0;
    double current_r_velocity = 0.0;
    
    double current_l_tl_error = 0.0;
    double current_r_tl_error = 0.0;

    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;

    double l_velocity_pid = 0.0;
    double r_velocity_pid = 0.0;
    double lscale = 0;
    double rscale = 0;

    double current_angular = 0.0;
    vector3D current_tl_velocity(0,0,0);
    vector3D prev_target_v(0,0,0);
    vector3D prev_target_r(0,0,0);
    vector3D v_fterm(0,0,0);
    vector3D r_fterm(0,0,0);
    double average_x_v = 0;
    double average_y_v = 0;

    uint64_t micros_now = -1;
    
    uint64_t micros_prev = pros::micros();
    uint64_t dt = -1;

    int32_t lu;
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    PID left_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    PID right_velocity_PID(velocity_kP, velocity_kI, velocity_kD);
    

    vector3D L2I_pos(WHEEL_BASE_RADIUS,0.0,0.0);
    while(true){
        while(toggleBase){
            left_angle = wrapAngle(getNormalizedSensorAngle(left_rotation_sensor)-90.0)*TO_RADIANS;
            right_angle = wrapAngle(getNormalizedSensorAngle(right_rotation_sensor)-90.0)*TO_RADIANS;
            current_left_vector = vector3D(cos(left_angle),sin(left_angle),0.0);
            current_right_vector = vector3D(cos(right_angle),sin(right_angle),0.0);

            current_l_velocity = ((luA.get_actual_velocity()+luB.get_actual_velocity()+llA.get_actual_velocity()+llB.get_actual_velocity())/4.0);
            current_r_velocity = ((ruA.get_actual_velocity()+ruB.get_actual_velocity()+rlA.get_actual_velocity()+rlB.get_actual_velocity())/4.0);

            current_angular = (current_l_velocity*sin(left_angle)+current_r_velocity*sin(right_angle))/(2.0*WHEEL_BASE_RADIUS);
            average_x_v = ((current_l_velocity*cos(left_angle))+(current_r_velocity*cos(right_angle)))/2.0;
            average_y_v = ((current_l_velocity*sin(left_angle))+(current_r_velocity*sin(right_angle)))/2.0;
            current_tl_velocity.load(average_x_v,average_y_v,0.0);

            prev_target_v = target_v;
            prev_target_r = target_r;
            // TODO: switch PID to go for target angle, switch actual to use current sensor angle
            target_v = normalizeJoystick(-leftX, -leftY).scalar(MAX_SPEED);
            target_r = normalizeRotation(rightX).scalar(MAX_ANGULAR);

            micros_prev = micros_now;
            micros_now = pros::micros();
            dt = micros_now-micros_prev;
            v_fterm = (target_v-prev_target_v)*(v_kF/dt);
            r_fterm = (target_r-prev_target_r)*(r_kF/dt);
            target_v = target_v + v_fterm;
            target_r = target_r + r_fterm;
            
            /*
            if(target_r.norm()<(MAX_ANGULAR*0.05) && current_angular>(MAX_ANGULAR*0.05)){
                target_r = vector3D(0,0,-0.3*current_angular);
            }
            if(target_v.norm()<(MAX_SPEED*0.05) && current_tl_velocity.norm()>(MAX_SPEED*0.05)){
                target_v = current_tl_velocity.scalar(0.1);

            pros::lcd::print(1,"r_fterm %3.3f", r_fterm.z);
            
            pros::lcd::print(4, "la_target %3.3f", (l_error+left_angle));
            pros::lcd::print(5, "ra_target %3.3f", (r_error+right_angle));

            pros::lcd::print(6, "rot_v_y %3.8f", rotational_v_vector.y);
            pros::lcd::print(7, "rot_v_x %3.8f", rotational_v_vector.x);

            pros::lcd::print(2, "la %3.3f", left_angle);
            pros::lcd::print(3, "ra %3.3f", right_angle);
            }*/

            rotational_v_vector = L2I_pos^target_r;
            
            v_left = target_v-rotational_v_vector;
            v_right = target_v+rotational_v_vector;

            bool reverse_right = false;
            bool reverse_left = false;
            
            // check if the angle is obtuse
            if (v_left * current_left_vector < 0){  
                // reverse if angle is obtuse for shorter rotation
                v_left = -v_left;
                reverse_left = true;
            }

            if (v_right * current_right_vector < 0){  
                // reverse if angle is obtuse for shorter rotation
                v_right = -v_right;
                reverse_right = true;
            }

            v_right_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_right*current_right_vector);
            v_left_velocity = SPEED_TO_RPM* TRANSLATE_RATIO*(v_left*current_left_vector);

            if(reverse_left){
                v_left_velocity = -v_left_velocity;
            }

            if(reverse_right){
                v_right_velocity = -v_right_velocity;
            }

            // calculate the error angle
            l_error = angle(current_left_vector, v_left);
            r_error = angle(current_right_vector, v_right);
            if (std::isnan(l_error) || std::isnan(r_error)) {
                l_error = 0.0; r_error = 0.0;
            }

            //calculate the wheel error
            current_l_tl_error = (v_left_velocity-current_l_velocity);
            current_r_tl_error = (v_right_velocity-current_r_velocity);

            l_velocity_pid += left_velocity_PID.step(current_l_tl_error);
            r_velocity_pid += right_velocity_PID.step(current_r_tl_error);

            // calculate the PID output
            l_angle_pid = left_angle_PID.step(l_error);
            r_angle_pid = right_angle_PID.step(r_error);

            lscale = scale * ((1.0-base_v)*fabs((l_error))+base_v);
            rscale = scale * ((1.0-base_v)*fabs((r_error))+base_v);

            lu = (int32_t)std::clamp(lscale * (l_velocity_pid + l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE); //this side seems less powerful on the robot
            ll = (int32_t)std::clamp(lscale * (l_velocity_pid - l_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
            ru = (int32_t)std::clamp(rscale * (r_velocity_pid + r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
            rl = (int32_t)std::clamp(rscale * (r_velocity_pid - r_angle_pid), -MAX_VOLTAGE, MAX_VOLTAGE);
        

            luA.move_voltage(lu);
            luB.move_voltage(lu);

            llA.move_voltage(ll);
            llB.move_voltage(ll);

            ruA.move_voltage(ru);
            ruB.move_voltage(ru);

            rlA.move_voltage(rl);
            rlB.move_voltage(rl);
        
            pros::delay(2);
        }
        pros::delay(10);
    }
}

void pivotWheels(double l_target_angle, double r_target_angle, double allowed_error) //pivot the left and right wheels by a certain amount
{
    allowed_error = fabs(allowed_error);
    double left_angle, right_angle; //numerical angle for each wheel in radians from -pi to pi
    //power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;

    //pivot angle error
    double l_angle_error = 9999999999;
    double r_angle_error =  9999999999;

    int32_t lu; //voltage variables for the four motor pairs of the base
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    
    while(fabs( l_angle_error) > allowed_error || fabs( r_angle_error) > allowed_error){ //while we havent reached the target pivot angles
        //get the current pivot angles of the left and right wheels in radians
        left_angle = (getNormalizedSensorAngle(left_rotation_sensor)) * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees
        right_angle = (getNormalizedSensorAngle(right_rotation_sensor)) * TO_RADIANS;
        //update pivot angle errors
        pros::lcd::print(0, "A");

        vector3D current_left_vector = vector3D(cos(left_angle),sin(left_angle),0.0);
        vector3D current_right_vector = vector3D(cos(right_angle),sin(right_angle),0.0);
        vector3D target_left_vector = vector3D(cos(l_target_angle),sin(l_target_angle),0.0);
        vector3D target_right_vector = vector3D(cos(r_target_angle),sin(r_target_angle),0.0);
        l_angle_error = angle(current_left_vector, target_left_vector);
        r_angle_error = angle(current_right_vector, target_right_vector);


        if (std::isnan(l_angle_error) || std::isnan(r_angle_error)) {
            l_angle_error = 0.0; r_angle_error = 0.0;
        }

        pros::lcd::print(4, "l_angle_error %lf", l_angle_error);
        pros::lcd::print(5, "r_angle_error %lf", r_angle_error);
        
        //calculate the PID output
        l_angle_pid = left_angle_PID.step(l_angle_error);
        r_angle_pid = right_angle_PID.step(r_angle_error);
        
        pros::lcd::print(6, "1_angle_pid %lf", l_angle_pid);
        pros::lcd::print(7, "r_angle_pid %lf", r_angle_pid);

        lu = (int32_t)(l_angle_pid * scale);//this side seems less powerful on the robot
        ll = (int32_t)(-l_angle_pid * scale);   
        ru = (int32_t)(r_angle_pid * scale);
        rl = (int32_t)(-r_angle_pid * scale);
        
        //calculate voltages required to run each motor, and scale them into the acceptable voltage range so they dont exceed max voltage
        //we have to scale the voltages because if we don't, it can happen that one or more motors dont move as fast as we expected because we ordered it to move
        //at a higher voltage than it can physically achieve, and this will throw off the proportions of velocity of the four motor pairs, and cause the robot
        //to move in unexpected ways. Scaling means that sometimes the robot moves slower than expected, but at least it moves correctly otherwise.
        clampVoltage(lu, ll, ru, rl); //ensure the voltages are within usable range

        luA.move_voltage(lu);
        luB.move_voltage(lu);

        llA.move_voltage(ll);
        llB.move_voltage(ll);

        ruA.move_voltage(ru);
        ruB.move_voltage(ru);

        rlA.move_voltage(rl);
        rlB.move_voltage(rl);
    
        pros::delay(5);
    }
    pros::lcd::print(0, "B");
    brake();
    pros::delay(5);
}

void rotateWheels(double l_distance, double r_distance, double allowed_error){ //rotate the left and right wheels by a certain amount WHILE maintaining pivot angle
    double l_distance_moved = 0.0; //distance that the left and right wheel moved
    double r_distance_moved = 0.0;

    double l_distance_error = l_distance; //distance error of the wheels
    double r_distance_error = r_distance;

    //pivot angles of the wheel at the start of the motion, we MUST maintain these pivot angles so the robot moves straight
    double l_angleMaintain = getNormalizedSensorAngle(left_rotation_sensor) * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees
    double r_angleMaintain = getNormalizedSensorAngle(right_rotation_sensor) * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees

    double left_angle, right_angle; //numerical angle for each wheel in radians from -pi to pi
    //steering angle error
    double l_error = 0.0;
    double r_error = 0.0;
    //power output for angle component of pid
    double l_angle_pid = 0.0;
    double r_angle_pid = 0.0;
    //power output for translation component of pid
    double l_distance_pid = 0.0;
    double r_distance_pid = 0.0;
    
    int32_t lu; //voltage variables for the four motor pairs of the base
    int32_t ll;
    int32_t ru;
    int32_t rl;

    PID left_angle_PID(angle_kP, angle_kI, angle_kD);
    PID right_angle_PID(angle_kP, angle_kI, angle_kD);
    PID left_distance_PID(distance_kP, distance_kI, distance_kD);
    PID right_distance_PID(distance_kP, distance_kI, distance_kD);

    tareBaseMotorEncoderPositions(); //tare all base motor encoder positions

    while(fabs(l_distance_error) > allowed_error || fabs(r_distance_error) > allowed_error){ //while the left and right wheels have not moved the target distance
        //get the distance that the robot has moved since we started this function
        //we have to divide by four because the velocity of the wheel is the average of the velocities of the four motors for that wheel
        //we cannot just take one motor for each gear of each wheel, since each gear is powered by two motors which could be running at marginally different speeds
        //by taking readings from all four motors of each wheel, we get more accurate results
        l_distance_moved = ((luA.get_position()+luB.get_position()+llA.get_position()+llB.get_position())/4.0) / ticks_per_mm;
        r_distance_moved = ((ruA.get_position()+ruB.get_position()+rlA.get_position()+rlB.get_position())/4.0) / ticks_per_mm;

        // std::cout << "....................................PID..................................." << std::endl;

        // std::cout << "l_distance" << l_distance << std::endl;
        // std::cout << "r_distance" << r_distance << std::endl;
        // std::cout << "l_distance_moved" << l_distance_moved << std::endl;
        // std::cout << "r_distance_moved" << r_distance_moved << std::endl;

        l_distance_error = l_distance - l_distance_moved; //calculate the error distance
        r_distance_error = r_distance - r_distance_moved;

        // std::cout << "l_distance_error" << l_distance_error << std::endl;
        // std::cout << "r_distance_error" << r_distance_error << std::endl;
        
        //get the current pivot angles of the left and right wheels in radians
        left_angle = getNormalizedSensorAngle(left_rotation_sensor) * TO_RADIANS; //note that the function getNormalizedSensorAngle already implements wrapAngle to bound the angle between -180 and 180 degrees
        right_angle = getNormalizedSensorAngle(right_rotation_sensor) * TO_RADIANS;

        // std::cout << "l_angleMaintain" << l_angleMaintain << std::endl;
        // std::cout << "r_angleMaintain" << r_angleMaintain << std::endl;
        // std::cout << "left_angle" << left_angle << std::endl;
        // std::cout << "right_angle" << right_angle << std::endl;

        // calculate the error angle
        vector3D l_target_angle = vector3D(cos(l_angleMaintain), sin(l_angleMaintain), 0);
        vector3D r_target_angle = vector3D(cos(r_angleMaintain), sin(r_angleMaintain), 0);
        vector3D l_current_angle = vector3D(cos(left_angle), sin(left_angle), 0);
        vector3D r_current_angle = vector3D(cos(right_angle), sin(right_angle), 0);

        l_error = angle(l_current_angle, l_target_angle);
        r_error = angle(r_current_angle, r_target_angle);

        // std::cout << "l_error" << l_error << std::endl;
        // std::cout << "r_error" << r_error << std::endl;

        //calculate the PID output
        l_angle_pid = left_angle_PID.step(l_error);
        r_angle_pid = right_angle_PID.step(r_error);

        // std::cout << "l_angle_pid" << l_angle_pid << std::endl;
        // std::cout << "r_angle_pid" << r_angle_pid << std::endl;

        l_distance_pid = left_distance_PID.step(l_distance_error);
        r_distance_pid = right_distance_PID.step(r_distance_error);

        // std::cout << "l_distance_pid" << l_distance_pid << std::endl;
        // std::cout << "r_distance_pid" << r_distance_pid << std::endl;

        lu = (int32_t)(l_distance_pid + l_angle_pid);//this side seems less powerful on the robot
        ll = (int32_t)(l_distance_pid - l_angle_pid);    
        ru = (int32_t)(r_distance_pid + r_angle_pid);
        rl = (int32_t)(r_distance_pid - r_angle_pid);

        luA.move_velocity(lu);
        luB.move_velocity(lu);

        llA.move_velocity(ll);
        llB.move_velocity(ll);

        ruA.move_velocity(ru);
        ruB.move_velocity(ru);

        rlA.move_velocity(rl);
        rlB.move_velocity(rl);
    
        pros::delay(5);
    }

    brake();
    pros::delay(5);
}

struct MotionStepCommand{ //contains the amount that the wheels should rotate and the target angle of the wheels
    double Lmove, Lpivot, Rmove, Rpivot;
    MotionStepCommand(double Lmove, double Lpivot, double Rmove, double Rpivot)
        : Lmove(Lmove), Lpivot(Lpivot), Rmove(Rmove), Rpivot(Rpivot) {}
};
struct StepCommandList{ //contains the list of commands for the base to follow in order to produce the path, AND contains the eight hermite coefficients for the left and right wheels
    std::vector<MotionStepCommand> Steps;
    double cax, cay, cbx, cby, ccx, ccy, cdx, cdy;
};
struct Waypoint{ //stores a waypoint of the auton path
    vector3D position, velocity;
    Waypoint(vector3D position, vector3D velocity)
        : position(position), velocity(velocity) {}
};

std::vector<Waypoint> ImportWaypointConfig(std::string config)
{
    std::vector<Waypoint> waypoints;
    if (config.empty())
        return waypoints;
    std::string xValue, yValue, magnitude, direction;
    size_t i = 0;
    while (i < config.length()) {
        try {
            if (config[i] == 'x') {
                i++; // skip over the x character
                while (config[i] != 'y') {
                    xValue += config[i];
                    i++;
                }
            }
            if (config[i] == 'y') {
                i++; // skip over the y character
                while (config[i] != 'v') {
                    yValue += config[i];
                    i++;
                }
            }
            if (config[i] == 'v') {
                i++; // skip over the v character
                while (config[i] != 't') {
                    magnitude += config[i];
                    i++;
                }
            }
            if (config[i] == 't') {
                i++; // skip over the t character
                while (config[i] != '&') {
                    direction += config[i];
                    i++;
                }
            }
            if (config[i] == '&') {
                i++; // skip over the & character
                //convert the velocity polar vector into a xy vector
                double directionvalue = std::stod(direction) * TO_RADIANS;
                directionvalue = directionvalue - (M_PI / 2);
                double velocityx = std::stod(magnitude) * std::cos(directionvalue);
                double velocityy = std::stod(magnitude) * std::sin(directionvalue);

                //construct the position and velocity vectors
                vector3D position = vector3D(std::stod(xValue), std::stod(yValue), 0);
                vector3D velocity = vector3D(velocityx, velocityy);

                waypoints.emplace_back( //construct a new waypoint and attach it to the back of the waypoints vector
                    position,
                    velocity
                );
                std::cout << "x position of waypoint" << position.x << std::endl;
                std::cout << "y position of waypoint" << position.y << std::endl;
                std::cout << "x value of velocity" << velocityx << std::endl;
                std::cout << "y value of velocity" << velocityy << std::endl;
                std::cout << "magnitude of velocity" << magnitude << std::endl;
                std::cout << "direction of velocity in radians" << directionvalue << std::endl;

                // reset the strings to be ready to translate the next waypoint
                xValue.clear();
                yValue.clear();
                magnitude.clear();
                direction.clear();
            }
        } catch (const std::exception&) {
            break;
        }
    }
    return waypoints;
}

struct Point{ // A struct to represent a single point in the lookup table
    double x; //u value
    double y; //angle value
    Point(double x, double y)
        : x(x), y(y) {}
};
double linearInterpolate(const std::vector<Point>& points, double x) { // Function to perform linear interpolation in the orientation lookup tables
    //Ensure the lookup table is sorted by x
    auto compare = [](const Point& a, const Point& b) { return a.x < b.x; };
    auto it = std::lower_bound(points.begin(), points.end(), Point{x, 0.0}, compare);
    // Handle cases where x is out of range
    if (it == points.begin()) {
        return points.front().y; // Return the first y-value if x is too small
    }
    if (it == points.end()) {
        return points.back().y; // Return the last y-value if x is too large
    }
    // Find the two points for interpolation
    auto p2 = *it;
    auto p1 = *(it - 1);
    double slope = (p2.y - p1.y) / (p2.x - p1.x);     // Perform linear interpolation
    return p1.y + slope * (x - p1.x);
}
std::vector<Point> generateLocalLookupTable(double startValue, double endValue, std::vector<Point> GlobalLookupTable, double resolution) //generates a subtable of a larger lookup table
{
    std::vector<Point> LocalLookupTable = {
        {}
    };
    LocalLookupTable.emplace_back(0, linearInterpolate(GlobalLookupTable, startValue)); //input the start point of the lookup table
    for(double i = 0.01; i < 0.99; i = i + (1 / resolution)) //input all the points in the middle of the lookup table
    //range of the for loop is from 0.01 < x < 0.99 so that we do not repeat the points at x = 0 and x = 1
        LocalLookupTable.emplace_back(i, linearInterpolate(GlobalLookupTable, i + startValue));
    LocalLookupTable.emplace_back(1, linearInterpolate(GlobalLookupTable, endValue)); //input the end point of the lookup table
    for(int i = 0; i < LocalLookupTable.size(); i++)
    {
        std::cout << "LocalLookupTable point " << i << " x value is " << LocalLookupTable[i].x << std::endl;
        std::cout << "LocalLookupTable point " << i << " y value is " << LocalLookupTable[i].y << std::endl;
    }
    return LocalLookupTable;
}

void GetNextStep(std::vector<MotionStepCommand>& Steps, vector3D NewRobotPosition, double NewRobotOrientation, vector3D PreviousLeftWheelPosition, vector3D PreviousRightWheelPosition) {
    std::cout << "newrobotposition.x" << NewRobotPosition.x << std::endl;
    std::cout << "newrobotposition.y" << NewRobotPosition.y << std::endl;
    std::cout << "newrobotorientation" << NewRobotOrientation << std::endl;
    std::cout << "prevlwheelpos.x" << PreviousLeftWheelPosition.x << std::endl;
    std::cout << "prevlwheelpos.y" << PreviousLeftWheelPosition.y << std::endl;
    std::cout << "prevrwheelpos.x" << PreviousRightWheelPosition.x << std::endl;
    std::cout << "prevrwheelpos.y" << PreviousRightWheelPosition.y << std::endl;

    //apply definition of L(t) and R(t) to get current left and right wheel position
    //left and right displacements are the relative positions of the left and right wheels relative to the robot
    double prevleftdispx, prevleftdispy, prevrightdispx, prevrightdispy;
    double acuteAngle; //the acute angle between the robot direction vector and the x axis

    if(NewRobotOrientation > -M_PI && NewRobotOrientation < -M_PI / 2)
    {
        acuteAngle = NewRobotOrientation + M_PI;
        prevleftdispx = sin(acuteAngle) * WHEEL_BASE_RADIUS;
        prevleftdispy = -cos(acuteAngle) * WHEEL_BASE_RADIUS;
        prevrightdispx = -sin(acuteAngle) * WHEEL_BASE_RADIUS;
        prevrightdispy = cos(acuteAngle) * WHEEL_BASE_RADIUS;
    }
    else if(NewRobotOrientation == -M_PI / 2)
    {
        prevleftdispx = WHEEL_BASE_RADIUS;
        prevleftdispy = 0;
        prevrightdispx = -WHEEL_BASE_RADIUS;
        prevrightdispy = 0;
    }
    else if(NewRobotOrientation > -M_PI / 2 && NewRobotOrientation < 0)
    {
        acuteAngle = -NewRobotOrientation;
        prevleftdispx = sin(acuteAngle) * WHEEL_BASE_RADIUS;
        prevleftdispy = cos(acuteAngle) * WHEEL_BASE_RADIUS;
        prevrightdispx = -sin(acuteAngle) * WHEEL_BASE_RADIUS;
        prevrightdispy = -cos(acuteAngle) * WHEEL_BASE_RADIUS;
    }
    else if(NewRobotOrientation == 0)
    {
        prevleftdispx = 0;
        prevleftdispy = WHEEL_BASE_RADIUS;
        prevrightdispx = 0;
        prevrightdispy = -WHEEL_BASE_RADIUS;
    }
    else if(NewRobotOrientation > 0 && NewRobotOrientation < M_PI / 2)
    {
        acuteAngle = NewRobotOrientation;
        prevleftdispx = -sin(acuteAngle) * WHEEL_BASE_RADIUS;
        prevleftdispy = cos(acuteAngle) * WHEEL_BASE_RADIUS;
        prevrightdispx = sin(acuteAngle) * WHEEL_BASE_RADIUS;
        prevrightdispy = -cos(acuteAngle) * WHEEL_BASE_RADIUS;
    }
    else if(NewRobotOrientation == M_PI / 2)
    {
        prevleftdispx = -WHEEL_BASE_RADIUS;
        prevleftdispy = 0;
        prevrightdispx = WHEEL_BASE_RADIUS;
        prevrightdispy = 0;
    }
    else if(NewRobotOrientation > M_PI / 2 && NewRobotOrientation < M_PI)
    {
        acuteAngle = M_PI - NewRobotOrientation;
        prevleftdispx = -sin(acuteAngle) * WHEEL_BASE_RADIUS;
        prevleftdispy = -cos(acuteAngle) * WHEEL_BASE_RADIUS;
        prevrightdispx = sin(acuteAngle) * WHEEL_BASE_RADIUS;
        prevrightdispy = cos(acuteAngle) * WHEEL_BASE_RADIUS;
    }
    else
    {
        prevleftdispx = 0;
        prevleftdispy = -WHEEL_BASE_RADIUS;
        prevrightdispx = 0;
        prevrightdispy = WHEEL_BASE_RADIUS;
    }
    
    double newlwheelposx = NewRobotPosition.x + prevleftdispx;
    double newlwheelposy = NewRobotPosition.y + prevleftdispy;
    double newrwheelposx = NewRobotPosition.x + prevrightdispx;
    double newrwheelposy = NewRobotPosition.y + prevrightdispy;

    std::cout << "leftdisp.x" << prevleftdispx << std::endl;
    std::cout << "leftdisp.y" << prevleftdispy << std::endl;
    std::cout << "rightdisp.x" << prevrightdispx << std::endl;
    std::cout << "rightdisp.y" << prevrightdispy << std::endl;

    std::cout << "newlwheelpos.x" << newlwheelposx << std::endl;
    std::cout << "newlwheelpos.y" << newlwheelposy << std::endl;
    std::cout << "newrwheelpos.x" << newrwheelposx << std::endl;
    std::cout << "newrwheelpos.y" << newrwheelposy << std::endl;

    //compare new left and right wheel positions to the previous left and right wheel positions to see how each wheel should move during the next step, to get from the previous to the new state
    double lstepx = newlwheelposx - PreviousLeftWheelPosition.x;
    double lstepy = newlwheelposy - PreviousLeftWheelPosition.y;
    double rstepx = newrwheelposx - PreviousRightWheelPosition.x;
    double rstepy = newrwheelposy - PreviousRightWheelPosition.y;
    vector3D LeftStep = vector3D(lstepx, lstepy);
    vector3D RightStep = vector3D(rstepx, rstepy);

    std::cout << "leftstepx" << lstepx << std::endl;
    std::cout << "leftstepy" << lstepy << std::endl;
    std::cout << "rightstepx" << rstepx << std::endl;
    std::cout << "rightstepy" << rstepy << std::endl;

    std::cout << "leftstepangle" << LeftStep.getAngle() << std::endl;
    std::cout << "rightstepangle" << RightStep.getAngle() << std::endl;
    std::cout << "leftstepdist" << LeftStep.magnitude() << std::endl;
    std::cout << "rightstepdist" << RightStep.magnitude() << std::endl;

    Steps.push_back(MotionStepCommand(LeftStep.magnitude(), LeftStep.getAngle(), RightStep.magnitude(), RightStep.getAngle())); //encode the step information into the Steps list
}

StepCommandList GenerateHermitePath(vector3D pStart, vector3D pEnd, vector3D vStart, vector3D vEnd, double StepLength, std::vector<Point> LocalOrientationLookupTable) {
    StepCommandList StepCL; //this list will store all the step motion data that causes the robot to execute the path

    //THIS PRODUCES THE HERMITE SPLINE COEFFICIENTS. THESE ARE NOT CONSTANTS FOR YOU TO TUNE. DO NOT CHANGE THESE CONSTANTS.
    StepCL.cax = (2.0 * pStart.x) + vStart.x - (2.0 * pEnd.x) + vEnd.x;
    StepCL.cay = (2.0 * pStart.y) + vStart.y - (2.0 * pEnd.y) + vEnd.y;
    StepCL.cbx = (-3.0 * pStart.x) - (2.0 * vStart.x) + (3.0 * pEnd.x) - vEnd.x;
    StepCL.cby = (-3.0 * pStart.y) - (2.0 * vStart.y) + (3.0 * pEnd.y) - vEnd.y;
    StepCL.ccx = vStart.x;
    StepCL.ccy = vStart.y;
    StepCL.cdx = pStart.x;
    StepCL.cdy = pStart.y;

    std::cout << "cax" << StepCL.cax << std::endl;
    std::cout << "cay" << StepCL.cay << std::endl;
    std::cout << "cbx" << StepCL.cbx << std::endl;
    std::cout << "cby" << StepCL.cby << std::endl;
    std::cout << "ccx" << StepCL.ccx << std::endl;
    std::cout << "ccy" << StepCL.ccy << std::endl;
    std::cout << "cdx" << StepCL.cdx << std::endl;
    std::cout << "cdy" << StepCL.cdy << std::endl;    

    vector3D ct[4] = { //this array of vector3D represents the coefficients of the parametric polynomial function C(t) which defines the curve of the path
        vector3D(StepCL.cax, StepCL.cay),
        vector3D(StepCL.cbx, StepCL.cby),
        vector3D(StepCL.ccx, StepCL.ccy),
        vector3D(StepCL.cdx, StepCL.cdy)
    };

    vector3D CurrentRobotPosition = pStart; //current robot position is simply the position of the robot at the start of the motion
    double CurrentRobotOrientation = LocalOrientationLookupTable[0].y;

    for (double t = StepLength; t < 1.0; t += StepLength) { //StepLength is a value to be tuned. Smaller steps produce a more accurate motion but PWM the motors more aggressively, slowing the motion down.
        //apply C(t) equation to get CurrentRobotPosition
        std::cout << "..........................................START NEW ITERATION................................................................" << std::endl;

        std::cout << "prevsteppositionx" << CurrentRobotPosition.x << std::endl;
        std::cout << "prevsteppositiony" << CurrentRobotPosition.y << std::endl;
        std::cout << "prevsteporientation" << CurrentRobotOrientation << std::endl;

        //update previous left and right wheel positions
        //apply definition of L(t) and R(t) to get current left and right wheel position

        //left and right displacements are the relative positions of the left and right wheels relative to the robot
        double prevleftdispx, prevleftdispy, prevrightdispx, prevrightdispy;
        double acuteAngle; //the acute angle between the robot direction vector and the x axis

        if(CurrentRobotOrientation > -M_PI && CurrentRobotOrientation < -M_PI / 2)
        {
            acuteAngle = CurrentRobotOrientation + M_PI;
            prevleftdispx = sin(acuteAngle) * WHEEL_BASE_RADIUS;
            prevleftdispy = -cos(acuteAngle) * WHEEL_BASE_RADIUS;
            prevrightdispx = -sin(acuteAngle) * WHEEL_BASE_RADIUS;
            prevrightdispy = cos(acuteAngle) * WHEEL_BASE_RADIUS;
        }
        else if(CurrentRobotOrientation == -M_PI / 2)
        {
            prevleftdispx = WHEEL_BASE_RADIUS;
            prevleftdispy = 0;
            prevrightdispx = -WHEEL_BASE_RADIUS;
            prevrightdispy = 0;
        }
        else if(CurrentRobotOrientation > -M_PI / 2 && CurrentRobotOrientation < 0)
        {
            acuteAngle = -CurrentRobotOrientation;
            prevleftdispx = sin(acuteAngle) * WHEEL_BASE_RADIUS;
            prevleftdispy = cos(acuteAngle) * WHEEL_BASE_RADIUS;
            prevrightdispx = -sin(acuteAngle) * WHEEL_BASE_RADIUS;
            prevrightdispy = -cos(acuteAngle) * WHEEL_BASE_RADIUS;
        }
        else if(CurrentRobotOrientation == 0)
        {
            prevleftdispx = 0;
            prevleftdispy = WHEEL_BASE_RADIUS;
            prevrightdispx = 0;
            prevrightdispy = -WHEEL_BASE_RADIUS;
        }
        else if(CurrentRobotOrientation > 0 && CurrentRobotOrientation < M_PI / 2)
        {
            acuteAngle = CurrentRobotOrientation;
            prevleftdispx = -sin(acuteAngle) * WHEEL_BASE_RADIUS;
            prevleftdispy = cos(acuteAngle) * WHEEL_BASE_RADIUS;
            prevrightdispx = sin(acuteAngle) * WHEEL_BASE_RADIUS;
            prevrightdispy = -cos(acuteAngle) * WHEEL_BASE_RADIUS;
        }
        else if(CurrentRobotOrientation == M_PI / 2)
        {
            prevleftdispx = -WHEEL_BASE_RADIUS;
            prevleftdispy = 0;
            prevrightdispx = WHEEL_BASE_RADIUS;
            prevrightdispy = 0;
        }
        else if(CurrentRobotOrientation > M_PI / 2 && CurrentRobotOrientation < M_PI)
        {
            acuteAngle = M_PI - CurrentRobotOrientation;
            prevleftdispx = -sin(acuteAngle) * WHEEL_BASE_RADIUS;
            prevleftdispy = -cos(acuteAngle) * WHEEL_BASE_RADIUS;
            prevrightdispx = sin(acuteAngle) * WHEEL_BASE_RADIUS;
            prevrightdispy = cos(acuteAngle) * WHEEL_BASE_RADIUS;
        }
        else
        {
            prevleftdispx = 0;
            prevleftdispy = -WHEEL_BASE_RADIUS;
            prevrightdispx = 0;
            prevrightdispy = WHEEL_BASE_RADIUS;
        }
        std::cout << "prevleftdispx" << prevleftdispx << std::endl;
        std::cout << "prevleftdispy" << prevleftdispy << std::endl;
        std::cout << "prevrightdispx" << prevrightdispx << std::endl;
        std::cout << "prevrightdispy" << prevrightdispy << std::endl;

        vector3D previous_left_displacement = vector3D(prevleftdispx, prevleftdispy);
        vector3D previous_right_displacement = vector3D(prevrightdispx, prevrightdispy);
        vector3D PreviousLeftWheelPosition = CurrentRobotPosition + previous_left_displacement;
        vector3D PreviousRightWheelPosition = CurrentRobotPosition + previous_right_displacement;

        vector3D prevRobotPosition = CurrentRobotPosition;

        //find new robot position
        CurrentRobotPosition = vector3D(
            ct[0].x * std::pow(t, 3) + ct[1].x * std::pow(t, 2) + ct[2].x * t + ct[3].x, //x polynomial of the parametric equation C(t)
            ct[0].y * std::pow(t, 3) + ct[1].y * std::pow(t, 2) + ct[2].y * t + ct[3].y //y polynomial of the parametric equation C(t)
        );
        std::cout << "newsteppositionx" << CurrentRobotPosition.x << std::endl;
        std::cout << "newsteppositiony" << CurrentRobotPosition.y << std::endl;
        
        CurrentRobotOrientation = linearInterpolate(LocalOrientationLookupTable, t);

        GetNextStep(StepCL.Steps, CurrentRobotPosition, CurrentRobotOrientation, PreviousLeftWheelPosition, PreviousRightWheelPosition);
    }
    return StepCL;
}

void move_auton(){ //execute full auton path
    //convert the config string into a big list of waypoints
    std::vector<Waypoint> waypoints = ImportWaypointConfig( //if waypoint velocity parameter is too small, the path will fail.
        "x500.0y500.0v1100.0t90.0&x1000.0y500.0v1100.0t90.0&x2000.0y2000.0v1500.0t0.0&");

    std::vector<Point> GlobalOrientationLookupTable = { //plots u value in parameter space of spline paths (represented as x value in the table) against orientation angle (represented as y value in the table) 
        {0, M_PI / 2},
        {1, M_PI / 2},
        {2, 0}};

    std::vector<Point> LocalOrientationLookupTable = {};

    int waypointIndex = 0; //note that this is the same as u value in parameter space
    while(waypointIndex < waypoints.size() - 1) //run until all paths have been executed
    {
        LocalOrientationLookupTable.clear();
        LocalOrientationLookupTable = generateLocalLookupTable(waypointIndex, waypointIndex + 1, GlobalOrientationLookupTable, 15);
        StepCommandList stepCommands = GenerateHermitePath( //generate the path (in the form of a list of step commands for the robot to follow) to get from the current waypoint to the next waypoint
            waypoints[waypointIndex].position, 
            waypoints[waypointIndex + 1].position, 
            waypoints[waypointIndex].velocity, 
            waypoints[waypointIndex + 1].velocity,
            0.1, //step length of the path (length in parameter space not real space)
            LocalOrientationLookupTable); //determines what orientation the robot has at any given point

        //execute the step command list to get from the current waypoint to the next waypoint
        for(int i = 0; i < (int)stepCommands.Steps.size(); i++){ //run until the path is fully executed
            MotionStepCommand current_command(stepCommands.Steps[i]); //get the current step command
            std::cout << "......................................START NEW STEP..................................." << std::endl;
            std::cout << "Lpivot" << current_command.Lpivot << std::endl;
            std::cout << "Rpivot" << current_command.Rpivot << std::endl;
            std::cout << "Lmove" << current_command.Lmove << std::endl;
            std::cout << "Rmove" << current_command.Rmove << std::endl;

            pivotWheels(current_command.Lpivot, current_command.Rpivot, 0.1); //steer the wheels to the correct angle
            rotateWheels(current_command.Lmove, current_command.Rmove, 10); //rotate the wheels the correct distance

            pros::delay(2);
        }
        waypointIndex++;

        pros::delay(5);
    }
}

void autonomous(){
    move_auton();
}

void initialize(){
    pros::lcd::initialize();
    luA.set_brake_mode(MOTOR_BRAKE_COAST);
    luB.set_brake_mode(MOTOR_BRAKE_COAST);
    llA.set_brake_mode(MOTOR_BRAKE_COAST);
    llB.set_brake_mode(MOTOR_BRAKE_COAST);
    ruA.set_brake_mode(MOTOR_BRAKE_COAST);
    ruB.set_brake_mode(MOTOR_BRAKE_COAST);
    rlA.set_brake_mode(MOTOR_BRAKE_COAST);
    rlB.set_brake_mode(MOTOR_BRAKE_COAST);
    liftL.set_brake_mode(MOTOR_BRAKE_HOLD);
    liftR.set_brake_mode(MOTOR_BRAKE_HOLD);

    //while(!left_rotation_sensor.reset());
    //while(!right_rotation_sensor.reset());

    left_rotation_sensor.set_data_rate(5);
    right_rotation_sensor.set_data_rate(5);

    left_rotation_sensor.set_position(0);
    right_rotation_sensor.set_position(0);

    pros::Task move_base(moveBase);
    //pros::Task serial_read(serialRead);

    //master.clear();
}

void opcontrol(){
    while(true){
        leftX = master.get_analog(ANALOG_LEFT_X);
        leftY = master.get_analog(ANALOG_LEFT_Y);
        rightX = master.get_analog(ANALOG_RIGHT_X);
        if(master.get_digital_new_press(DIGITAL_B)) autonomous();
        if(master.get_digital_new_press(DIGITAL_A)) toggleBase = !toggleBase;

        pros::delay(5);
    }
}