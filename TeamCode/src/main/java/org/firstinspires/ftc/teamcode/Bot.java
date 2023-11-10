package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Bot {

    public enum BotState {
        INTAKE, // surgical tubing ready to pick up pixel
        STORAGE_FULL, // either 1 or 2 pixels in storage
        STORAGE_NOT_FULL,
        OUTTAKE, // ready to outtake
    }

    public static Bot instance;
    private final DcMotorEx fl, fr, bl, br;
 //   private final Servo tcServo, droneServo_1, droneServo_2, outtakeServo;

    public BotState state = BotState.STORAGE_NOT_FULL;

    public OpMode opMode;

    public boolean fieldCentricRunMode = false;

    //verify

    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
        }
        return instance;
    }

    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    private Bot(OpMode opMode) {
        this.opMode = opMode;
        enableAutoBulkRead();
        //what is this
        try {
            fieldCentricRunMode = false;
        } catch (Exception e) {
            fieldCentricRunMode = false;

        }


        fl = opMode.hardwareMap.get(DcMotorEx.class, "fl");
        fr = opMode.hardwareMap.get(DcMotorEx.class, "fr");
        bl = opMode.hardwareMap.get(DcMotorEx.class, "bl");
        br = opMode.hardwareMap.get(DcMotorEx.class, "br");



        fl.setMode(RUN_USING_ENCODER);
    }

    public void reverseMotors(){
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void prepForOuttake() {
        state = BotState.STORAGE_FULL;
        //slides.runToBottom(); code in slides subsystem
    }
    //move to slides

    public void outtake() { // must be combined with bot.slide.run___() in MainTeleOp
        state = BotState.OUTTAKE;
    }

    public void secure() {
        //pipeline detected pixel, and intake was run
    }


    public void fixMotors(double velocity) {
        fl.setVelocity(-velocity); //invert
        fr.setVelocity(velocity);
        bl.setVelocity(-velocity); // invert
        br.setVelocity(velocity);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                forwardBackSpeed - strafeSpeed - turnSpeed,
                forwardBackSpeed + strafeSpeed + turnSpeed,
                forwardBackSpeed + strafeSpeed - turnSpeed,
                forwardBackSpeed - strafeSpeed + turnSpeed
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.setPower(speeds[0]);
        fr.setPower(speeds[1]);
        bl.setPower(speeds[2]);
        br.setPower(speeds[3]);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed, double heading) {
        double magnitude = Math.sqrt(strafeSpeed * strafeSpeed + forwardBackSpeed * forwardBackSpeed);
        double theta = (Math.atan2(forwardBackSpeed, strafeSpeed) - heading) % (2 * Math.PI);
        double[] speeds = {
                magnitude * Math.sin(theta + Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) - turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta + Math.PI / 4) - turnSpeed
        };

        double maxSpeed = 0;

        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }

        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }

        fl.setPower(speeds[0]);
        fr.setPower(speeds[1]);
        bl.setPower(speeds[2]);
        br.setPower(speeds[3]);
    }

    private void enableAutoBulkRead() {
        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }


    //Hello there!!!!
    public void resetEncoder() {
        fl.setMode(STOP_AND_RESET_ENCODER);
        fr.setMode(STOP_AND_RESET_ENCODER);
        br.setMode(STOP_AND_RESET_ENCODER);
        bl.setMode(STOP_AND_RESET_ENCODER);
    }

}