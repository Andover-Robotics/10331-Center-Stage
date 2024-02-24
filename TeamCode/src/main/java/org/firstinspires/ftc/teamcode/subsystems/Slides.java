package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.MotionProfiler;

/*

problems:

1. sometimes slides stop working when trying to switch to preset position
2. sometimes when going to pre-set posotion, one of the motors cant reach there and keeps whirring


 */



public class Slides {
    public final MotorEx midMotor, rightMotor;
    private double target = 0 ;
    private int encoderTickPerLevel = -650;
    private final static double p = 0.015, i = 0 , d = 0, f = 0, staticF = 0.25;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1, powerMin = 0.1;
    public double power;
    private double manualPower;
    public boolean goingDown;
    private final double MIN_POWER = 0.1;


    public static final double MAX_VELOCITY = 30000, MAX_ACCELERATION = 20000;
    //tune
    private PIDFController controller;
    private MotionProfiler profiler = new MotionProfiler(30000, 20000);
    private double profile_init_time = 0;


    public enum slidesPosition{
        GROUND,
        LOW,
        MID,
        HIGH
    }
    private slidesPosition position = slidesPosition.GROUND;

    public static int storage = 0, top = -2250, mid = -1000, low = -500;
    //tune high and storage

    private final OpMode opMode;


    public Slides(OpMode opMode) {
        rightMotor = new MotorEx(opMode.hardwareMap, "slidesRight", Motor.GoBILDA.RPM_312);
        midMotor = new MotorEx(opMode.hardwareMap, "slidesCenter", Motor.GoBILDA.RPM_312);

        rightMotor.setInverted(true);
        midMotor.setInverted(false);


        //right is the one closest to outtake
        //left and center are the chain


        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);


        rightMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        midMotor.setRunMode(Motor.RunMode.RawPower);
        midMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        this.opMode = opMode;
    }


    public void runTo(double pos) {
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);



        midMotor.setRunMode(Motor.RunMode.RawPower);
        midMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.init(rightMotor.getCurrentPosition(), pos);
        profile_init_time = opMode.time;


        goingDown = pos > target;
        target = pos;
    }


    public void runToTop() {
//        if(rightMotor.getCurrentPosition() >= top)
//            return;
        runTo(top);
        position = slidesPosition.HIGH;
    }


    public void runToMid() {
//        if(position != position.MID) runTo(mid);
        runTo(mid);
        position = slidesPosition.MID;
    }


    public void runToLow() {
//        if(position != position.LOW) runTo(low);
        runTo(low);
        position = slidesPosition.LOW;
    }
//if start with preset position nothing works
    //start with manual preset does not work

    public void runToStorage() {
        position = slidesPosition.GROUND;
        runTo(storage);
//        if(rightMotor.getCurrentPosition() <= storage)
//            return;
//        else
//            position = slidesPosition.GROUND;
    }




    public void runToManual(double manual) {
        if(rightMotor.getCurrentPosition() >= storage && manual>0) {
            manualPower = 0;
            return;
        }
        if(rightMotor.getCurrentPosition() <= top && manual<0) {
            manualPower = 0;
            return;
        }

        /*
        if(rightMotor.getCurrentPosition() == storage && manualPower > 0) {
            manualPower = 0;
            return;
        }
         */

        if (manual > powerMin || manual < - powerMin)
            manualPower = manual;
        else
            manualPower = 0;
    }

    public int getCurrentPosition() {
        return rightMotor.getCurrentPosition();
    }

    public void resetEncoder() {
        rightMotor.resetEncoder();
        midMotor.resetEncoder();
    }


    public void periodicWithoutProfiler() {
        rightMotor.setInverted(true);
        midMotor.setInverted(false);

        controller.setPIDF(p, i, d, f);


        if (manualPower != 0) {
            midMotor.set(manualPower / manualDivide);
            rightMotor.set(manualPower / manualDivide);
        }
        else {
            power = staticF * controller.calculate(rightMotor.getCurrentPosition());
            rightMotor.set(manualPower);
            if (manualPower < Math.abs(0.1)) {
                midMotor.set(0);
            } else {
                midMotor.set(manualPower);
            }
        }
    }

    public void periodic() {
        rightMotor.setInverted(true);
        midMotor.setInverted(false);

        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.profile_pos(dt));
            power = powerUp * controller.calculate(rightMotor.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(rightMotor.getCurrentPosition());
            }
            rightMotor.set(power);
            midMotor.set(power);


        } else {
            if (profiler.isDone()) {
                profiler = new MotionProfiler(30000, 20000);
            }

            if (manualPower != 0) {
                controller.setSetPoint(rightMotor.getCurrentPosition());
                rightMotor.set(manualPower / manualDivide);
                midMotor.set(manualPower / manualDivide);

            } else {
                power = staticF * controller.calculate(rightMotor.getCurrentPosition());
                rightMotor.set(power);

                if (power < Math.abs(0.1)) {
                    midMotor.set(0);
                } else {
                    midMotor.set(power);
                }
            }
        }
    }

    public void test(double power) {
        rightMotor.set(power);
        midMotor.set(power);
    }


    public void resetProfiler(){
        profiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION);
    }


    public double getManualPower(){
        return manualPower;
    }

}