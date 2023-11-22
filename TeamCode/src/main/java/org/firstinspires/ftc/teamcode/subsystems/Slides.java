package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.MotionProfiler;


public class Slides {
    public final MotorEx slidesMotor;
    private double current_pos = 0 ;
    private final static double p = 0.015, i = 0 , d = 0, f = 0, staticF = 0.25;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1;
    public double power;
    private double manualPower;
    public boolean goingDown;
    private final double MIN_POWER = 0.1;

    public static final double MAX_VELOCITY = 30000, MAX_ACCELERATION = 20000;
    //tune
    private PIDFController controller;
    private MotionProfiler profiler;
    private double profile_init_time = 0;

    public enum slidesPosition{
        GROUND,
        LOW,
        MID_1,
        MID_2,
        HIGH
    }
    private slidesPosition position = slidesPosition.GROUND;

    public static int storage = 0, top = 3250, mid_1 = 1700, mid_2 = 1000, low = 450;
    //tune

    private final OpMode opMode;

    public Slides(OpMode opMode) {
        this.opMode = opMode;
        slidesMotor = new MotorEx(opMode.hardwareMap, "slides motor");
        slidesMotor.setInverted(true);
        slidesMotor.setRunMode(Motor.RunMode.RawPower);

        controller = new PIDFController(p,i,d,f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);

        profiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION);
    }

    public void runTo(double target) {
        slidesMotor.setRunMode(Motor.RunMode.RawPower);
        slidesMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setInverted(true);

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);

        resetProfiler();
        profiler.init(slidesMotor.getCurrentPosition(), target);
        profile_init_time = opMode.time;

        //goingDown =  targetPoint > target;
        goingDown =  target > current_pos;
        current_pos = target;
    }

    public void runToTop() {
        runTo(top);
        position = slidesPosition.HIGH;
    }

    public void runToMid(int num) {
        if(num==1){
            runTo(mid_1);
            position = slidesPosition.MID_1;
        }
        else {
            runTo(mid_2);
            position = slidesPosition.MID_2;
        }
    }

    public void runToLow() {
        runTo(low);
        position = slidesPosition.LOW;
    }
    public void runToStorage() {
        runTo(storage);
        position = slidesPosition.GROUND;
    }


    public void runToManual(double power){
        if(Math.abs(power) > MIN_POWER) {
            manualPower = power;
        }
        else {
            manualPower = 0;
        }
    }


    public void resetEncoder() {
        slidesMotor.resetEncoder();
    }

    public void periodic() {
        slidesMotor.setInverted(true);
        slidesMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        //if motion profiling is being used i.e, runTo() methods

        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.profile_pos(dt));
            power = powerUp * controller.calculate(slidesMotor.getCurrentPosition());

            if (goingDown) {
                power = powerDown * controller.calculate(slidesMotor.getCurrentPosition());
            }

            slidesMotor.set(power);
        }

        /*all the cases in which isOver is true
        a) we were using runTo() control, but we successfully ran to the target value and the trajectory is now over
        b) we are using manual control
         */

        else {
            if (profiler.isDone()) profiler = new MotionProfiler(30000, 20000);
            //if we aren't using manual power, but the profile just ended, we should create a new motionprofiler obj to
            //erase previous trajectory data

            if (manualPower != 0) {
                //controller.setSetPoint(slidesMotor.getCurrentPosition());
                slidesMotor.set(manualPower / manualDivide);
            } else {
                slidesMotor.set(0);

                //pls work bro :praying:
                /*
                power = staticF * controller.calculate(slidesMotor.getCurrentPosition());
                slidesMotor.set(power);
                 */
                //THIS IS WHY it's going back to original position after we let go of the joystick
                //the setPoint was set to the position BEFORE it moved manually.

                /*
                if (power < Math.abs(0.1)) slidesMotor.set(0);
                else slidesMotor.set(power);
                 */
            }
        }
    }

    public void resetProfiler(){
        profiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION);
    }

    public double getManualPower(){
        return manualPower;
    }

}

