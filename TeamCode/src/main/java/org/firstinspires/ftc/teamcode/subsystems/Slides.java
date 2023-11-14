package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MotionProfiler;


public class Slides {
    public final DcMotorEx slidesMotor;

    private double targetPoint = 0 ;

    //ABSOLUTELY HAVE TO TUNE!!!!
    private final static double p = 0.015, i = 0 , d = 0, f = 0, staticF = 0.25;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1;
    public double power;

    public double manualPower;
    public boolean goingDown;
    private final double MIN_POWER = 0.1;
    public static final double MAX_VELOCITY = 1150 * 5.2, MAX_ACCELERATION = 30;

    private PIDFController controller;
    private final PIDCoefficients coeff = new PIDCoefficients(p,i,d);
    private final PIDFCoefficients pidfcoeff = new PIDFCoefficients(p,i,d,f);
    private MotionProfiler profiler;
    private double profile_init_time = 0;


    public enum slidesPosition{
        GROUND,
        LOW,
        MID,
        HIGH
    }
    private slidesPosition position = slidesPosition.GROUND;

    public static int storage = 0, top = -2200, mid = -1350, low = -600;
    //all these values are probably wrong

    private final OpMode opMode;

    public Slides(OpMode opMode) {
        this.opMode = opMode;

        slidesMotor = opMode.hardwareMap.get(DcMotorEx.class, "slides motor");
        slidesMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfcoeff);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(p,i,d,f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
    }

    public void runTo(double target) {
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.init(slidesMotor.getCurrentPosition(), target);
        profile_init_time = opMode.time;

        goingDown =  targetPoint > target;
        targetPoint = target;
    }

    public void runToTop() {
        runTo(top);
        position = slidesPosition.HIGH;
    }

    public void runToMid() {
        runTo(mid);
        position = slidesPosition.MID;
    }

    public void runToBottom() {
        runTo(top);
        position = slidesPosition.HIGH;
    }

    /*
    public void runToMid2(int stage) {
        runTo(top);
        position = slidesPosition.HIGH;
    }

     */

    public void runToManual(double power){
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        if(Math.abs(power) > MIN_POWER) {
            slidesMotor.setPower(power);
            telemetry.addData("Slide is running at this power", power);
        }
        else {
            slidesMotor.setPower(MIN_POWER);
            telemetry.addLine("Slide is running at minimum power (0.1)");
        }
    }

    public void brake(){
        slidesMotor.setPower(0);
    }


    public void resetEncoder() {
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void periodic() {
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.profile_pos(dt));
            power = powerUp * controller.calculate(slidesMotor.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(slidesMotor.getCurrentPosition());
            }
            slidesMotor.setPower(power);
        } else {
                profiler = new MotionProfiler(30000, 20000);

            if (manualPower != 0) {
                controller.setSetPoint(slidesMotor.getCurrentPosition());
                slidesMotor.setPower(manualPower / manualDivide);
            } else {
                power = staticF * controller.calculate(slidesMotor.getCurrentPosition());
                slidesMotor.setPower(power);
                if (power < Math.abs(0.1)) {
                    slidesMotor.setPower(0);
                } else {
                    slidesMotor.setPower(power);
                }
            }
        }
    }

    public void resetProfiler(){
        profiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION);
    }


}