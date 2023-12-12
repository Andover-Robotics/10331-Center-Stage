package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Bot.BotState.STORAGE_NOT_FULL;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Box;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Fourbar;
import org.firstinspires.ftc.teamcode.subsystems.Noodles;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.AprilTagsPipeline;
import org.openftc.easyopencv.OpenCvCamera;


public class Bot {

    public enum BotState {
        INTAKE, // surgical tubing ready to pick up pixel
        STORAGE_FULL, // 2 pixels in storage
        STORAGE_NOT_FULL, //1 or 0 pixels in storage
        OUTTAKE, //is in outtake position
    }

    public OpMode opMode;
    public BotState currentState = STORAGE_NOT_FULL;
    public static Bot instance;

    public OpenCvCamera camera;
    public AprilTagsPipeline aprilTagsPipeline;

   // public static AprilTagsDetection detections;

    public Slides slides;
    public Fourbar fourbar;

    public Noodles noodles;
    public Drone drone;

    public Box box;

    // public static DistanceSensor distanceSensor;

    private final DcMotorEx FL, FR, BL, BR;


    public boolean fieldCentricRunMode = false;
    private double distanceFromBackdrop;
    private final double optimalDistanceFromBackdrop = 10;
    //arbitrary number for now

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

        // distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");


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





    public void outtake(boolean pixelTwo, int stage){
        currentState = BotState.OUTTAKE;
        slides.runTo(stage);
        fourbar.outtake();
        box.depositFirstPixel();
        if(pixelTwo){
            box.depositSecondPixel();
            resetOuttake();
        }
    }


    public void storageSlides(){
        slides.runTo(1);
    }

    public void resetOuttake(){
        box.resetBox();
        storageSlides();
        fourbar.storage();
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

    public void resetEverything(){
        noodles.stop();
        reverseMotors();
        resetEncoder();
        slides.runToLow();
        fourbar.storage();
        box.resetBox();
    }

    private void enableAutoBulkRead() {
        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public void intake(){
        currentState = BotState.INTAKE;
        //box.resetBox();
        box.runWheel(false);
        noodles.intake();
    }
    public void stopIntake(){
        currentState = BotState.STORAGE_FULL;
        box.secure();
        noodles.stop();
    }






    public void resetEncoder() {
        FL.setMode(STOP_AND_RESET_ENCODER);
        FR.setMode(STOP_AND_RESET_ENCODER);
        BR.setMode(STOP_AND_RESET_ENCODER);
        BL.setMode(STOP_AND_RESET_ENCODER);
        slides.resetEncoder();
    }





   /* public void distanceTuning(DistanceSensor sensor){
        double diffy = this.distanceFromBackdrop - optimalDistanceFromBackdrop;
        boolean inRange = Math.abs(diffy) <= 5;
        if(inRange){
            return;
        }
        while(!inRange){
            if(diffy<0){
                back();
            }else{
                forward();
            }
            distanceFromBackdrop = sensor.getDistance(DistanceUnit.CM);
            diffy = distanceFromBackdrop - optimalDistanceFromBackdrop;
            inRange = Math.abs(diffy) <= 5;
            distanceTuning(sensor);
        }
    }

    */
/*
    public void aprilTagTuning(){
        AprilTagsDetection.detectTag();
        distanceFromBackdrop = detections.calcDistToTag();
        double diffy = this.distanceFromBackdrop - optimalDistanceFromBackdrop;
        boolean inRange = Math.abs(diffy) <= 5;
        while(!inRange){
            if(diffy<0){
                back();
            }else{
                forward();
            }
            distanceFromBackdrop = detections.calcDistToTag();
            diffy = distanceFromBackdrop - optimalDistanceFromBackdrop;
            inRange = Math.abs(diffy) <= 5;
        }
    }

 */


}