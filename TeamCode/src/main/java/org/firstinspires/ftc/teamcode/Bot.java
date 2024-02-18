package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Bot.BotState.STORAGE_NOT_FULL;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.TeamPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Box;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Fourbar;
import org.firstinspires.ftc.teamcode.subsystems.Noodles;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


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

    public OpenCvWebcam camera;
  //  public AprilTagsPipeline aprilTagsPipeline;

   // private static AprilTagsDetection detections;

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
        //what is this
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


      /*  FL.setMode(RUN_USING_ENCODER);
        FR.setMode(RUN_USING_ENCODER);
        BL.setMode(RUN_USING_ENCODER);
        BR.setMode(RUN_USING_ENCODER);

       */

        this.slides = new Slides(opMode);
        this.fourbar = new Fourbar(opMode);
        this.noodles = new Noodles(opMode);
        this.box = new Box(opMode);
    //    this.drone= new Drone(opMode);

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

    public void setIndividualMotorPower(int motor, double power) {
        //1 FL 2 FR 3 BL 4 BR
        switch(motor) {
            case 1:
                FL.setPower(power);
                break;
            case 2:
                FR.setPower(power);
            case 3:
                BL.setPower(power);
            case 4:
                BR.setPower(power);
        }
    }

    public void resetEverything(){
        noodles.stop();
        reverseMotors();
        resetEncoder();
      //  slides.runToLow();
        fourbar.storage();
        box.resetBox();
    }

    private void enableAutoBulkRead() {
        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }




    public void resetEncoder() {
        FL.setMode(STOP_AND_RESET_ENCODER);
        FR.setMode(STOP_AND_RESET_ENCODER);
        BR.setMode(STOP_AND_RESET_ENCODER);
        BL.setMode(STOP_AND_RESET_ENCODER);
      //  slides.resetEncoder();
    }

    public void initCamera(TeamPropDetectionPipeline pipeline){

        WebcamName camName = opMode.hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }



}
