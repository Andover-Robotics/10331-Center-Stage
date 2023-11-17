package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Slides2 {

    public final MotorEx slidesMotor;
    private PIDFController controller;
    private final OpMode opMode;
    public static double p = 0.04, i = 0, d = 0, f = 0, staticF = 0.25;

    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1, powerMin = 0.1;
    public double manualPower = 0;
    public static int storage = 0, top = 3250, mid_1 = 1700, mid_2 = 1000, low = 450;
    private double profile_init_time = 0;
    private boolean goingDown = false;
    private double target = 0;

    private MotionProfiler2 profiler = new MotionProfiler2(8000, 8000);

    public Slides2(OpMode opMode) {
        slidesMotor = new MotorEx(opMode.hardwareMap, "slides motor", Motor.GoBILDA.RPM_1150);
        slidesMotor.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        slidesMotor.setRunMode(Motor.RunMode.RawPower);
        slidesMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.opMode = opMode;
    }

    public void runTo(int t) {
        slidesMotor.setRunMode(Motor.RunMode.RawPower);
        slidesMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.init_new_profile(slidesMotor.getCurrentPosition(), t);
        profile_init_time = opMode.time;
        if (t > target) {
            goingDown = true;
        } else {
            goingDown = false;
        }
        target = t;
    }


    public void runToTop() {
        runTo(top);
    }

    public void runToMid(int num) {
        if(num==1){
            runTo(mid_1);
        }
        else {
            runTo(mid_2);
        }
    }

    public void runToLow() {
        runTo(low);
    }
    public void runToStorage() {
        runTo(storage);
    }

    public void runManual(double power) {
        if (power > powerMin || power < -powerMin) {
            manualPower = power;
        } else {
            manualPower = 0;
        }
    }

    public void periodic() {
        slidesMotor.setInverted(false);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.motion_profile_pos(dt));
            double power = powerUp * controller.calculate(slidesMotor.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(slidesMotor.getCurrentPosition());
            }
            slidesMotor.set(power);
        } else {
            if (profiler.isDone()) {
                profiler = new MotionProfiler2(30000, 20000);
            }
            if (manualPower != 0) {
                controller.setSetPoint(slidesMotor.getCurrentPosition());
                slidesMotor.set(manualPower / manualDivide);
            } else {
                double power = staticF * controller.calculate(slidesMotor.getCurrentPosition());
                slidesMotor.set(power);
            }
        }
    }

    public void resetEncoder() {
        slidesMotor.resetEncoder();
    }

    public int getPosition() {
        return slidesMotor.getCurrentPosition();
    }

    public void resetProfiler() {
        profiler = new MotionProfiler2(3000, 6000);
    }


}
