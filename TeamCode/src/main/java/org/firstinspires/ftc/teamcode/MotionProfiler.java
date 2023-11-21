package org.firstinspires.ftc.teamcode;

public class MotionProfiler {

    public MotionProfiler(double max_velocity, double max_acceleration){
        this.max_acceleration = max_acceleration;
        this.max_velocity = max_velocity;
    }
    private final double max_velocity, max_acceleration;
    private boolean isOver = true;
    //isOver is only false when the motion profile has been created and is not finished.
    // If we're using manual power in slides then we don't use the motion profiler, so isOver is true.
    private boolean isDone = false;
    private double temp_max_a, temp_max_v;
    private double start_pos, final_pos, distance, acceleration_dt, halfway_distance,
            acceleration_distance, new_max_velocity, deacceleration_dt, cruise_distance, cruise_dt,
            decel_time, entire_dt;

    public void init(double start_pos, double final_pos){
        this.start_pos = start_pos;
        this.final_pos = final_pos;
        isOver = false;

        distance = final_pos-start_pos;

        if(distance < 0){
            temp_max_v = -max_velocity;
            temp_max_a = -max_acceleration;
        }else{
            temp_max_a = max_acceleration;
            temp_max_v = max_velocity;
        }

        // calculate the time it takes to accelerate to max velocity
        acceleration_dt = temp_max_v / temp_max_a;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        halfway_distance = distance / 2;
        acceleration_distance = 0.5 * temp_max_a * acceleration_dt * acceleration_dt;

        if (Math.abs(acceleration_distance) > Math.abs(halfway_distance)) {
            acceleration_dt = Math.sqrt(Math.abs(halfway_distance / (0.5 * temp_max_a)));
        }
        acceleration_distance = 0.5 * temp_max_a * acceleration_dt * acceleration_dt;

        // recalculate max velocity based on the time we have to accelerate and decelerate
        new_max_velocity = temp_max_a * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / new_max_velocity;
        decel_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
    }


    public double profile_pos(double current_dt) {
//        Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.

        if (current_dt > entire_dt) {
            isOver = true;
            isDone = true;
            return final_pos;
        }

        // if we're accelerating
        if (current_dt < acceleration_dt)
            // use the kinematic equation for acceleration
            return start_pos + 0.5 * temp_max_a * Math.pow(current_dt,2);

            // if we're cruising
        else if (current_dt < decel_time) {
            acceleration_distance = 0.5 * temp_max_a * acceleration_dt * acceleration_dt;
            double cruise_current_dt = current_dt - acceleration_dt;

            // use the kinematic equation for constant velocity
            return start_pos + acceleration_distance + new_max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * temp_max_a * acceleration_dt * acceleration_dt;
            cruise_distance = new_max_velocity * cruise_dt;
            decel_time = current_dt - decel_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return start_pos + acceleration_distance + cruise_distance + new_max_velocity * decel_time - 0.5 * temp_max_a * decel_time * decel_time;
        }
    }

    public boolean isOver() {
        return isOver;
    }

    public boolean isDone() {
        return isDone;
    }
}



