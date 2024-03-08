package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Bot.BotState.STORAGE_NOT_FULL;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Box;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Fourbar;
import org.firstinspires.ftc.teamcode.subsystems.Noodles;
import org.firstinspires.ftc.teamcode.subsystems.Slides;


public class Bot {

    public enum BotState {
        INTAKE, // surgical tubing ready to pick up pixel
        STORAGE_FULL, // 2 pixels in storage
        STORAGE_NOT_FULL, //1 or 0 pixels in storage
    }

    public OpMode opMode;
    public BotState currentState = STORAGE_NOT_FULL;
    public static Bot instance;

    //public OpenCvCamera camera;
   // public AprilTagsPipeline aprilTagsPipeline;

   // public static AprilTagsDetection detections;

      public Slides slides;
      public Fourbar fourbar;

      public Noodles noodles;
      public Drone drone;

      public Box box;

    private final DcMotorEx FL, FR, BL, BR;


    public boolean fieldCentricRunMode = false;
    public double heading = 0.0;

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
        try {
            fieldCentricRunMode = false;
        } catch (Exception e) {
            fieldCentricRunMode = false;

        }

        FL = opMode.hardwareMap.get(DcMotorEx.class, "fl");
        FR = opMode.hardwareMap.get(DcMotorEx.class, "fr");
        BL = opMode.hardwareMap.get(DcMotorEx.class, "bl");
        BR = opMode.hardwareMap.get(DcMotorEx.class, "br");

        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        FL.setMode(RUN_USING_ENCODER);
        FR.setMode(RUN_USING_ENCODER);
        BL.setMode(RUN_USING_ENCODER);
        BR.setMode(RUN_USING_ENCODER);

        this.slides = new Slides(opMode);
        this.fourbar = new Fourbar(opMode);
        this.noodles = new Noodles(opMode);
        this.box = new Box(opMode);
        this.drone= new Drone(opMode);

    }

    public void reverseMotors(){
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
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
        FL.setPower(speeds[0]);
        FR.setPower(speeds[1]);
        BL.setPower(speeds[2]);
        BR.setPower(speeds[3]);

    }



    public void driveFieldCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
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

        //        for (int i = 0; i < 4; i++) {
        //            driveTrainMotors[i].set(speeds[i]);
        //        }
        // manually invert the left side

        FL.setPower(speeds[0]);
        FR.setPower(speeds[1]);
        BL.setPower(speeds[2]);
        BR.setPower(speeds[3]);
    }


    public void fixMotors() {
        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void resetEverything(){

        noodles.stop();
        reverseMotors();
        resetEncoder();
        slides.runToStorage();
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fourbar.storage();
        box.resetBox();
    }

    private void enableAutoBulkRead() {
        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    /*
    multithreading for intake:
     */
    Thread boxMovement = new Thread(() -> {
        if(currentState == BotState.INTAKE)
            box.runWheel(false);
        else
            box.secure();

    }
    );
    Thread noodlesMovement = new Thread(() -> {
        if(currentState == BotState.INTAKE)
            noodles.intake();
        else
            noodles.stop();
    }
    );
    public void intake(){
        currentState = BotState.INTAKE;
        boxMovement.start();
        noodlesMovement.start();
    }
    public void stopIntake(){
        currentState = BotState.STORAGE_FULL;
        boxMovement.start();
        noodlesMovement.start();
    }

    public void resetEncoder() {
        FL.setMode(STOP_AND_RESET_ENCODER);
        FR.setMode(STOP_AND_RESET_ENCODER);
        BR.setMode(STOP_AND_RESET_ENCODER);
        BL.setMode(STOP_AND_RESET_ENCODER);
        slides.resetEncoder();
    }

    public double getFLCurrent(){
        return FL.getCurrent(CurrentUnit.AMPS);
    }
    public double getFRCurrent(){
        return FR.getCurrent(CurrentUnit.AMPS);
    }
    public double getBLCurrent(){
        return BR.getCurrent(CurrentUnit.AMPS);
    }
    public double getBRCurrent(){
        return BR.getCurrent(CurrentUnit.AMPS);
    }
    public double getFLPower(){
        return FL.getPower();
    }
    public double getFRPower(){
        return FR.getPower();
    }
    public double getBLPower(){
        return BR.getPower();
    }
    public double getBRPower(){
        return BR.getPower();
    }

    public void strafeRight(){
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setPower(0.1);
        FR.setPower(0.1);
        BR.setPower(0.1);
        BL.setPower(0.1);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

    }
    public void strafeLeft(){

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setPower(0.1);
        FR.setPower(0.1);
        BR.setPower(0.1);
        BL.setPower(0.1);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);


    }


}