package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.MotionProfiler;


public class Slides {
    public final MotorEx leftMotor, midMotor, rightMotor;
    private double target = 0 ;
    private int encoderTickPerLevel = -650;
    private final static double p = 0.015, i = 0 , d = 0, f = 0, staticF = 0.25;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1;
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

    public static int storage = 100, top = 700 , mid = 500, low = 300;
    //tune

    private final OpMode opMode;

    public Slides(OpMode opMode) {
        leftMotor = new MotorEx(opMode.hardwareMap, "slidesLeft", Motor.GoBILDA.RPM_312);
        rightMotor = new MotorEx(opMode.hardwareMap, "slidesRight", Motor.GoBILDA.RPM_312);
        midMotor = new MotorEx(opMode.hardwareMap, "slidesCenter", Motor.GoBILDA.RPM_312);

        rightMotor.setInverted(false);
        leftMotor.setInverted(false);
        midMotor.setInverted(false);

        //right is the one closest to outtake
        //left and mid are the chain

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        midMotor.setRunMode(Motor.RunMode.RawPower);
        midMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.opMode = opMode;
    }

    public void runTo(double pos) {
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        midMotor.setRunMode(Motor.RunMode.RawPower);
        midMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.init(rightMotor.getCurrentPosition(), pos);
        profile_init_time = opMode.time;

        goingDown = pos > target;
        target = pos;
       // periodic();
    }

    public void runToTop() {
        runTo(top);
        position = slidesPosition.HIGH;
    }

    public void runToMid() {
        runTo(mid);
        position = slidesPosition.MID;
    }

    public void runToLow() {
        runTo(low);
        position = slidesPosition.LOW;
    }

    public void runToStorage() {
        runTo(storage);
        position = slidesPosition.GROUND;
    }


    public void runToManual(double power) {
        if(Math.abs(power) > MIN_POWER) {
            manualPower = power;
        }
        else {
            manualPower = 0;
        }
    }

    public void resetEncoder() {
        rightMotor.resetEncoder();
    }

    public void periodic() {
        rightMotor.setInverted(true);
        leftMotor.setInverted(true);
        midMotor.setInverted(true);

        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;

        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.profile_pos(dt));
            power = powerUp * controller.calculate(rightMotor.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(rightMotor.getCurrentPosition());
            }
            leftMotor.set(power);
            rightMotor.set(power);
            midMotor.set(power);

        } else {
            if (profiler.isDone()) {
                profiler = new MotionProfiler(30000, 20000);
            }

            if (manualPower != 0) {
                controller.setSetPoint(rightMotor.getCurrentPosition());
                midMotor.set(manualPower / manualDivide);
                rightMotor.set(manualPower / manualDivide);
                leftMotor.set(manualPower / manualDivide);

            } else {
                power = staticF * controller.calculate(rightMotor.getCurrentPosition());
                rightMotor.set(power);
                leftMotor.set(power);
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
        leftMotor.set(power);
        midMotor.set(power);

    }

    public void resetProfiler(){
        profiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION);
    }

    public double getManualPower(){
        return manualPower;
    }

}