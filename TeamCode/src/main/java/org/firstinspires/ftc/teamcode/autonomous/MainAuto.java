package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Bot;


/*
To Do:

1) TEST AUTOPATHS AND TELEOP!!!!
    - if splines do not work, switch to forward(), strafeRight(), and strafeLeft()
2) add more autopaths
3) odometry

 */

@Config
@Autonomous(name = "MainAutonomous")

public class MainAuto extends LinearOpMode{


    Bot bot;
    double distanceFromObject;

    enum Side {
        RED, BLUE, NULL
    }
    enum DistanceToBackdrop{
        CLOSE, FAR, NULL
    }

    //different paths to follow depending on driver input before match
    enum AutoPath{
        MECHANICAL_FAILURE, NO_SENSE, OPTIMAL
    }

  //  private TeamProp teamPropLocation = TeamProp.NOTDETECTED;

    Side side = Side.BLUE;
    DistanceToBackdrop dtb= DistanceToBackdrop.CLOSE;
    AutoPath autopath = AutoPath.MECHANICAL_FAILURE;


    public static double fx = 1078.03779;
    public static double fy = 1084.50988;
    public static double cx = 580.850545;
    public static double cy = 245.959325;

    // UNITS ARE METERS
    public static double tagSize = 0.032;


    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        //different start positions depending on alliance and distance from backdrop
        Pose2d startPoseBlueFar = new Pose2d(-52, 52, 0);
        Pose2d startPoseBlueClose = new Pose2d(38, 56, -90);
        Pose2d startPoseRedFar = new Pose2d(-52, -48, 0);
        Pose2d startPoseRedClose = new Pose2d(10, -52, 0);

        //writing variables for Vector2d positions that are reused
        Vector2d parkingPosBlue = new Vector2d(56,56);
        Vector2d parkingPosRed = new Vector2d(56,-56);
        Vector2d scoreBlue = new Vector2d(42,38);
        Vector2d scoreRed = new Vector2d(42,-34);



        //CAMERA STUFF =====================

        /*
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        bot.camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        TeamPropDetectionPipeline teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry);
        bot.aprilTagsPipeline= new AprilTagsPipeline(tagSize, fx, fy, cx, cy);


        bot.camera.setPipeline(teamPropDetectionPipeline);
        bot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                bot.camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        should be moved into a method to make it more succinct

         */


            /*
            SIDE:
                b=red
                a=blue
            DTB:
                X= Close
                Y= Far
             */


            //creating Trajectories/Paths
            TrajectorySequence blueAllianceFarRobotFail = drive.trajectorySequenceBuilder(startPoseBlueFar)
                    .splineTo(new Vector2d(-34,38), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .forward(-7)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreBlue,Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(parkingPosBlue,Math.toRadians(0))
                    .build();

            TrajectorySequence blueAllianceCloseRobotFail = drive.trajectorySequenceBuilder(startPoseBlueClose)
                    .splineTo(new Vector2d(10,38), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreBlue,Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(parkingPosBlue,Math.toRadians(0))
                    .build();

            TrajectorySequence redAllianceFarRobotFail = drive.trajectorySequenceBuilder(startPoseRedFar)
                    .splineTo(new Vector2d(-34,-34), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .forward(-10)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreRed,Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(parkingPosRed,Math.toRadians(-90))
                    .build();

            TrajectorySequence redAllianceCloseRobotFail = drive.trajectorySequenceBuilder(startPoseRedClose)
                    .splineTo(new Vector2d(15,-34),Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreRed,Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(parkingPosRed,Math.toRadians(0))
                    .build();

            TrajectorySequence blueAllianceFar = drive.trajectorySequenceBuilder(startPoseBlueFar)
                    .splineTo(new Vector2d(-34,38), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .forward(-7)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreBlue, Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                    .waitSeconds(1)
                    .splineTo(parkingPosBlue, Math.toRadians(0))
                    .build();

            TrajectorySequence redAllianceFar= drive.trajectorySequenceBuilder(startPoseRedFar)
                    .splineTo(new Vector2d(-34,-34), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .forward(-10)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreRed, Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                    .waitSeconds(1)
                    .splineTo(parkingPosRed, Math.toRadians(-90))
                    .build();

            TrajectorySequence blueAllianceClose = drive.trajectorySequenceBuilder(startPoseBlueClose)
                    .splineTo(new Vector2d(10,38), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreBlue, Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                    .waitSeconds(1)
                    .splineTo(parkingPosBlue, Math.toRadians(90))
                    .build();

            TrajectorySequence redAllianceClose= drive.trajectorySequenceBuilder(startPoseRedClose)
                    .splineTo(new Vector2d(15,-34), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreRed, Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                    .waitSeconds(1)
                    .splineTo(parkingPosRed, Math.toRadians(0))
                    .build();

            TrajectorySequence redAllianceCloseNoSense = drive.trajectorySequenceBuilder(startPoseRedClose)
                    .splineTo(new Vector2d(15,-34), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreRed, Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::scoreNoSense)
                    .waitSeconds(1)
                    .splineTo(parkingPosRed, Math.toRadians(0))
                    .build();

            TrajectorySequence blueAllianceCloseNoSense = drive.trajectorySequenceBuilder(startPoseBlueClose)
                    .splineTo(new Vector2d(10,38), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreBlue, Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::scoreNoSense)
                    .waitSeconds(1)
                    .splineTo(parkingPosBlue, Math.toRadians(90))
                    .build();

            TrajectorySequence blueAllianceFarNoSense = drive.trajectorySequenceBuilder(startPoseBlueFar)
                    .splineTo(new Vector2d(-34,38), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .forward(-7)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreBlue, Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::scoreNoSense)
                    .waitSeconds(1)
                    .splineTo(parkingPosBlue, Math.toRadians(0))
                    .build();

            TrajectorySequence redAllianceFarNoSense= drive.trajectorySequenceBuilder(startPoseRedFar)
                    .splineTo(new Vector2d(-34,-34), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .forward(-10)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreRed, Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::scoreNoSense)
                    .waitSeconds(1)
                    .splineTo(parkingPosRed, Math.toRadians(-90))
                    .build();

            while (!isStarted()) {
                gp1.readButtons();
                if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                    telemetry.addLine("Alliance: red");
                    side = Side.RED;
                    telemetry.update();
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                    telemetry.addLine("Alliance: blue");
                    side = Side.BLUE;
                    telemetry.update();
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                    telemetry.addLine("Distance: close");
                    dtb = DistanceToBackdrop.CLOSE;
                    telemetry.update();
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                    telemetry.addLine("Distance: far");
                    dtb = DistanceToBackdrop.FAR;
                    telemetry.update();
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    telemetry.addLine("Mode: Mechanical Failure");
                    autopath = AutoPath.MECHANICAL_FAILURE;
                    telemetry.update();
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    telemetry.addLine("Mode: No Sense");
                    autopath = AutoPath.NO_SENSE;
                    telemetry.update();
                }

                if (dtb == DistanceToBackdrop.FAR) {
                    //far positions
                    if (side == Side.RED) {
                        drive.setPoseEstimate(startPoseRedFar);
                    } else {
                        drive.setPoseEstimate(startPoseBlueFar);
                    }
                } else {
                    //close positions
                    if (side == Side.RED) {
                        drive.setPoseEstimate(startPoseRedClose);
                    } else {
                        drive.setPoseEstimate(startPoseBlueClose);
                    }
                }
            }
                waitForStart();
                if (opModeIsActive() && !isStopRequested()) {
                    telemetry.addLine("Distance:" + dtb);
                    telemetry.addLine("Alliance:" + side);
                    telemetry.addLine("Mode" + autopath);
                    telemetry.update();

                    if (dtb == DistanceToBackdrop.FAR) {
                        //blue side far
                        if (side == Side.BLUE) {
                            if (autopath == AutoPath.OPTIMAL) {
                                findTeamPropLocation();
                                drive.followTrajectorySequence(blueAllianceFar);
                            } else if (autopath == AutoPath.NO_SENSE) {
                                drive.followTrajectorySequence(blueAllianceFarNoSense);
                            } else {
                                drive.followTrajectorySequence(blueAllianceFarRobotFail);
                            }
                            //red side far
                        } else {
                            if (autopath == AutoPath.OPTIMAL) {
                                findTeamPropLocation();
                                drive.followTrajectorySequence(redAllianceFar);
                            } else if (autopath == AutoPath.NO_SENSE) {
                                drive.followTrajectorySequence(redAllianceFarNoSense);
                            } else {
                                drive.followTrajectorySequence(redAllianceFarRobotFail);
                            }
                        }

                    } else if (dtb == DistanceToBackdrop.CLOSE) {
                        //blue side close
                        if (side == Side.BLUE) {
                            if (autopath == AutoPath.OPTIMAL) {
                                findTeamPropLocation();
                                drive.followTrajectorySequence(blueAllianceClose);
                            } else if (autopath == AutoPath.NO_SENSE) {
                                drive.followTrajectorySequence(blueAllianceCloseNoSense);
                            } else {
                                drive.followTrajectorySequence(blueAllianceCloseRobotFail);
                            }
                            //red side close

                        } else {
                            if (autopath == AutoPath.OPTIMAL) {
                                findTeamPropLocation();
                                drive.followTrajectorySequence(redAllianceClose);
                            } else if (autopath == AutoPath.NO_SENSE) {
                                drive.followTrajectorySequence(redAllianceCloseNoSense);
                            } else {
                                drive.followTrajectorySequence(redAllianceCloseRobotFail);
                            }
                        }
                    }
                }
            }

    private void dropPurplePixel(){

        /*if(teamPropLocation==TeamProp.ONLEFT){
            bot.turn(-0.25);
        }
        else if(teamPropLocation==TeamProp.ONRIGHT){
            bot.turn(0.25);
        }
        else{
            bot.forward();
        }
        */

        bot.forward();
        bot.noodles.reverseIntake();
        //note: java code execution happens very fast, so having .reverseIntake()
        // immediately followed by .stop() in the same method will not be effective.


        telemetry.addData("purple pixel is currently being dropped",".");
        telemetry.update();
    }

    public void stopNoodles(){
        bot.noodles.stop();

        telemetry.addData("noodles are stopped",".");
        telemetry.update();
    }


    private void findTeamPropLocation(){
        /*if(TeamPropDetectionPipeline.teamPropLocation== TeamProp.ONLEFT){
            teamPropLocation= TeamProp.ONLEFT;
        }

        else if(TeamPropDetectionPipeline.teamPropLocation== TeamProp.ONRIGHT){
            teamPropLocation= TeamProp.ONRIGHT;
        }
        else if(TeamPropDetectionPipeline.teamPropLocation== TeamProp.MIDDLE){
            teamPropLocation= TeamProp.MIDDLE;
        }
        else{
            teamPropLocation= TeamProp.NOTDETECTED;
        }

         */
        telemetry.addData("Team prop should be located here",".");
        telemetry.update();
    }

    private void stageScore(){
        //score in stage area (lit just reversing intake)
        bot.noodles.reverseIntake();
        telemetry.addData("Scoring in stage area should occur right now",".");
        telemetry.update();
    }


    private void senseAndScore(){
        //locates and moves to corresponding position on Backdrop based on april tags
        //switch to aprilTagsPipeline => looking for AprilTags

       /* bot.camera.setPipeline(bot.aprilTagsPipeline);
        int counter=0;

        //based on where team prop is, move to the corresponding position on the backdrop

        try{
            if(teamPropLocation== TeamProp.ONLEFT){
                //keep strafing left until robot detects AprilTag or if you have run loop over 5 times
                while(AprilTagsDetection.tagOfInterest.id!= 1 && counter<5){
                    AprilTagsDetection.detectTag();
                    bot.strafeLeft();
                    counter++;
                    if(counter==4){
                        throw new AprilTagException("april tag could not be located");
                    }
                }
            }
            else if(teamPropLocation== TeamProp.ONRIGHT){

                while(AprilTagsDetection.tagOfInterest.id!=3 && counter<5){
                    AprilTagsDetection.detectTag();
                    bot.strafeRight();
                    counter++;
                    if(counter==4){
                        throw new AprilTagException("april tag could not be located");
                    }
                }
            }else if(teamPropLocation != TeamProp.NOTDETECTED){
                throw new PropException("the prop wasn't detected");
            }
            bot.outtake(1,true);

        }catch(Exception e){
            e.printStackTrace();
        }

        */
        telemetry.addData("Sensing and scoring should occur right now", ".");
        telemetry.update();
    }

    private void scoreNoSense(){
        //bot.outtake(true,1);
        telemetry.addData("Scoring with no sensing should occur right now",".");
        telemetry.update();
    }
}
