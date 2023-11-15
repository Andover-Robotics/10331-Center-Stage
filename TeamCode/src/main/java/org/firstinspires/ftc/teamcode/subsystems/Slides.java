package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.MotionProfiler;


public class Slides {
    public final MotorEx slidesMotor;

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
    private MotionProfiler profiler;
    private double profile_init_time = 0;

    public enum slidesPosition{
        GROUND,
        LOW,
        MID,
        HIGH
    }
    private slidesPosition position = slidesPosition.GROUND;

    public static int storage=0, top = 3250, mid = 1700, low = 450;

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

    public void runToLow() {
        runTo(low);
        position = slidesPosition.LOW;
    }
    public void runToStorage() {
        runTo(storage);
        position = slidesPosition.GROUND;
    }


    public void runToManual(double power){
//        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        if(Math.abs(power) > MIN_POWER) {
            manualPower = power;
        }
        else {
            manualPower = 0;
        }
    }

    public void brake(){
        slidesMotor.set(0);
    }


    public void resetEncoder() {
        slidesMotor.resetEncoder();
    }

    public void periodic() {
        slidesMotor.setInverted(false);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.profile_pos(dt));
            power = powerUp * controller.calculate(slidesMotor.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(slidesMotor.getCurrentPosition());
            }
            slidesMotor.set(power);
        } else {
            if (profiler.isDone())
                profiler = new MotionProfiler(30000, 20000);

            if (manualPower != 0) {
                controller.setSetPoint(slidesMotor.getCurrentPosition());
                slidesMotor.set(manualPower / manualDivide);
            } else {
                power = staticF * controller.calculate(slidesMotor.getCurrentPosition());
                slidesMotor.set(power);
                if (power < Math.abs(0.1)) {
                    slidesMotor.set(0);
                } else {
                    slidesMotor.set(power);
                }
            }
        }
    }

    public void resetProfiler(){
        profiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION);
    }
}