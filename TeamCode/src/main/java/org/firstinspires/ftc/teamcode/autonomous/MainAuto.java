package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.autonomous.test.TeamPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@Autonomous(name = "MainAutonomous")

/*

1) Why is it stopping after reaching spike mark and turning (running intake whole time)
        - possible error in DropPurplePixel() when detecting team prop
2) Go further in first movement
3) Test all auto positions

 */

public class MainAuto extends LinearOpMode {

    //optimal has 3 diff paths
    // optimal through middle has 3 diff paths
    // robot fail has 3 diff paths
    // robot fail through middle has 3 paths
    // no sense has 1 path
    // just park has 1 path

    // total => 14 x 4 = 56 paths



    Bot bot;


    double distanceFromObject;
    boolean wait=false;
    boolean throughMiddle= true;

    enum Side {
        RED, BLUE,
    }
    enum DistanceToBackdrop {
        CLOSE, FAR,
    }

    //different paths to follow depending on driver input before match
    enum AutoPath{
        MECHANICAL_FAILURE, OPTIMAL, NO_SENSE, JUST_PARK
    }


    Side side = Side.BLUE;
    DistanceToBackdrop dtb= DistanceToBackdrop.CLOSE;
    AutoPath autopath = AutoPath.OPTIMAL;
    TeamPropDetectionPipeline.TeamProp prop;
    private ElapsedTime time = new ElapsedTime();


    public static double fx = 1078.03779;
    public static double fy = 1084.50988;
    public static double cx = 580.850545;
    public static double cy = 245.959325;

    TrajectorySequence strafeLeft;
    TrajectorySequence strafeRight;
    TrajectorySequence forward;

    Pose2d currentPose;

    // UNITS ARE METERS
    public static double tagSize = 0.032;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        GamepadEx gp1 = new GamepadEx(gamepad1);

       // different start positions depending on alliance and distance from backdrop
        Pose2d startPoseBlueFar = new Pose2d(-36, 52, Math.toRadians(-90));
        Pose2d startPoseBlueClose = new Pose2d(10, 56, Math.toRadians(-90));
        Pose2d startPoseRedClose = new Pose2d(10, -52, Math.toRadians(90));
        Pose2d startPoseRedFar = new Pose2d(-34, -48, Math.toRadians(90));



        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        //writing variables for Vector2d positions that are reused
        Vector2d parkingPosBlue = new Vector2d(56,56);
        Vector2d parkingPosRed = new Vector2d(56,-56);
        Vector2d scoreBlue = new Vector2d(42,38);
        Vector2d scoreRed = new Vector2d(42,-34);



        //CAMERA STUFF =====================


        WebcamName camName = hardwareMap.get(WebcamName.class, "webcam");
        bot.camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        TeamPropDetectionPipeline teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry);
        //  bot.aprilTagsPipeline= new AprilTagsPipeline(tagSize, fx, fy, cx, cy);


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

        prop= teamPropDetectionPipeline.getTeamPropLocation();


            /*
            SIDE:
                b=red
                a=blue
            DTB:
                X= Close
                Y= Far
             */


        //robot fail has no outtake
        //optimal outtakes on the backdrop
        // no sense does not involve team prop sensing or april tag sensing (still outtakes)


        TrajectorySequence blueAllianceClose = drive.trajectorySequenceBuilder(startPoseBlueClose)
                .forward(20)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(90))
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                .waitSeconds(1)
                .strafeLeft(24)
                .forward(20)
                .waitSeconds(1)
                .build();

        TrajectorySequence blueAllianceCloseRobotFail = drive.trajectorySequenceBuilder(startPoseBlueClose)
                .forward(20)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(90))
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .strafeLeft(24)
                .forward(20)
                .waitSeconds(1)
                .build();

        TrajectorySequence blueAllianceCloseJustPark = drive.trajectorySequenceBuilder(startPoseBlueClose)
                .strafeLeft(42)
                .build();

        TrajectorySequence blueAllianceFar = drive.trajectorySequenceBuilder(startPoseBlueFar)
                .forward(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .strafeLeft(80)
                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                .waitSeconds(1)
                .strafeLeft(24)
                .forward(20)
                .waitSeconds(1)
                .build();

        TrajectorySequence blueAllianceFarThroughMiddle = drive.trajectorySequenceBuilder(startPoseBlueFar)
                .forward(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .forward(30)
                .strafeLeft(80)
                .turn(Math.toRadians(90))
                .strafeLeft(30)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                .waitSeconds(1)
                .strafeLeft(24)
                .forward(20)
                .waitSeconds(1)
                .build();


        TrajectorySequence blueAllianceFarRobotFail = drive.trajectorySequenceBuilder(startPoseBlueFar)
                .forward(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .strafeLeft(80)
                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .strafeLeft(24)
                .forward(20)
                .waitSeconds(1)
                .build();

        TrajectorySequence blueAllianceFarThroughMiddleRobotFail = drive.trajectorySequenceBuilder(startPoseBlueFar)
                .forward(18)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .forward(30)
                .strafeLeft(80)
                .turn(Math.toRadians(90))
                .strafeLeft(30)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,this::stageScore)
                .waitSeconds(1)
                .strafeLeft(24)
                .forward(20)
                .waitSeconds(1)
                .build();

        TrajectorySequence blueAllianceFarJustPark = drive.trajectorySequenceBuilder(startPoseBlueFar)
                .forward(48)
                .strafeLeft(80)
                .back(54)
                .strafeLeft(20)
                .build();


        TrajectorySequence redAllianceFar = drive.trajectorySequenceBuilder(startPoseRedFar)
                .forward(13)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(-90))
                .forward(80)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                .waitSeconds(1)
                .strafeRight(25)
                .forward(10)
                .build();

        TrajectorySequence redAllianceFarThroughMiddle = drive.trajectorySequenceBuilder(startPoseRedFar)
                .forward(18)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .forward(30)
                .strafeRight(80)
                .turn(Math.toRadians(-90))
                .strafeRight(30)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                .waitSeconds(1)
                .strafeRight(25)
                .forward(10)
                .build();


        TrajectorySequence redAllianceFarRobotFail = drive.trajectorySequenceBuilder(startPoseRedFar)
                .forward(13)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(-90))
                .forward(80)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .strafeRight(25)
                .forward(10)
                .build();

        TrajectorySequence redAllianceFarThroughMiddleRobotFail = drive.trajectorySequenceBuilder(startPoseRedFar)
                .forward(18)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .forward(30)
                .strafeRight(80)
                .turn(Math.toRadians(-90))
                .strafeRight(30)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,this::stageScore)
                .waitSeconds(1)
                .strafeRight(25)
                .forward(10)
                .build();


        TrajectorySequence redAllianceFarJustPark = drive.trajectorySequenceBuilder(startPoseRedFar)
                .forward(48)
                .strafeRight(80)
                .back(54)
                .strafeRight(20)
                .build();


        TrajectorySequence redAllianceClose = drive.trajectorySequenceBuilder(startPoseRedClose)
                .forward(20)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(-90))
                .forward(36)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,this::senseAndScore)
                .waitSeconds(1)
                .strafeRight(27)
                .forward(20)
                .build();


        TrajectorySequence redAllianceCloseRobotFail = drive.trajectorySequenceBuilder(startPoseRedClose)
                .forward(20)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(-90))
                .forward(36)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .strafeRight(27)
                .forward(20)
                .build();

        TrajectorySequence redAllianceCloseJustPark = drive.trajectorySequenceBuilder(startPoseRedClose)
                .strafeRight(42)
                .build();

        strafeLeft = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(5)
                .build();
        strafeRight = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(5)
                .build();
        forward = drive.trajectorySequenceBuilder(startPose)
                .forward(2)
                .build();


       /*     TrajectorySequence redAllianceFar= drive.trajectorySequenceBuilder(startPoseRedFar)
                    .splineTo(new Vector2d(-34,-34), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .forward(-10)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreRed, Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::scoreNoSense)
                    .waitSeconds(1)
                    .splineTo(parkingPosRed, Math.toRadians(-90))
                    .build();


            TrajectorySequence redAllianceClose= drive.trajectorySequenceBuilder(startPose)
                    .splineTo(new Vector2d(15,-34), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                    .waitSeconds(1.5)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                    .splineTo(scoreRed, Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::scoreNoSense)
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

        */


        while (!isStarted()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                telemetry.addLine("Alliance: red");
                side = Side.RED;
                teamPropDetectionPipeline.setAlliance(1);
                telemetry.update();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                telemetry.addLine("Alliance: blue");
                side = Side.BLUE;
                teamPropDetectionPipeline.setAlliance(2);
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
            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                telemetry.addLine("Mode: No Sense");
                autopath = AutoPath.NO_SENSE;
                telemetry.update();
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                telemetry.addLine("Mode: Optimal");
                autopath = AutoPath.OPTIMAL;
                telemetry.update();
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                telemetry.addLine("Mode: Just Park");
                autopath = AutoPath.JUST_PARK;
                telemetry.update();
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.START)){
                telemetry.addLine("wait for 15 seconds before outtaking");
                wait=true;
                telemetry.update();
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                telemetry.addLine("not through middle");
                throughMiddle=false;
                telemetry.update();
            }



        }
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            if (dtb == DistanceToBackdrop.FAR) {
                if (side == Side.BLUE) {
                    drive.setPoseEstimate(startPoseBlueFar);
                    if (autopath == AutoPath.OPTIMAL) {
                        if(throughMiddle){
                            drive.followTrajectorySequence(blueAllianceFarThroughMiddle);
                        }
                        else {
                            drive.followTrajectorySequence(blueAllianceFar);
                        }
                    }
                    else if(autopath== AutoPath.JUST_PARK){
                        drive.followTrajectorySequence(blueAllianceFarJustPark);
                    }
                    else {
                        if(throughMiddle){
                            drive.followTrajectorySequence(blueAllianceFarThroughMiddleRobotFail);
                        }
                        else {
                            drive.followTrajectorySequence(blueAllianceFarRobotFail);
                        }
                    }
                }
                else {
                    drive.setPoseEstimate(startPoseRedFar);
                    if (autopath == AutoPath.OPTIMAL) {
                        if(throughMiddle){
                            drive.followTrajectorySequence(redAllianceFarThroughMiddle);
                        }
                        else{
                            drive.followTrajectorySequence(redAllianceFar);
                        }
                    }
                    else if(autopath== AutoPath.JUST_PARK){
                        drive.followTrajectorySequence(redAllianceFarJustPark);
                    }
                    else {
                        if(throughMiddle){
                            drive.followTrajectorySequence(redAllianceFarThroughMiddleRobotFail);
                        }
                        else {
                            drive.followTrajectorySequence(redAllianceFarRobotFail);
                        }
                    }
                }
            }
            else if (dtb == DistanceToBackdrop.CLOSE) {
                if (side == Side.BLUE) {
                    drive.setPoseEstimate(startPoseBlueClose);
                    if (autopath == AutoPath.OPTIMAL) {
                        drive.followTrajectorySequence(blueAllianceClose);
                    }
                    else if(autopath== AutoPath.JUST_PARK){
                        drive.followTrajectorySequence(blueAllianceCloseJustPark);
                    }
                    else {
                        drive.followTrajectorySequence(blueAllianceCloseRobotFail);
                    }

                }
                else {
                    drive.setPoseEstimate(startPoseRedClose);
                    if (autopath == AutoPath.OPTIMAL) {
                        drive.followTrajectorySequence(redAllianceClose);
                    }
                    else if(autopath== AutoPath.JUST_PARK){
                        drive.followTrajectorySequence(redAllianceCloseJustPark);
                    }
                    else {
                        drive.followTrajectorySequence(redAllianceCloseRobotFail);
                    }
                }
            }
        }
    }

    private void dropPurplePixel(){
        if(autopath==AutoPath.NO_SENSE){
            bot.noodles.reverseIntake();
        }
        else {
            currentPose = drive.getPoseEstimate();
            if (prop == TeamPropDetectionPipeline.TeamProp.ONLEFT) {
                drive.turn(Math.toRadians(90));
                bot.noodles.reverseIntake();
                drive.turn(Math.toRadians(-90));
            } else if (prop == TeamPropDetectionPipeline.TeamProp.ONRIGHT) {
                drive.turn(Math.toRadians(-90));
                bot.noodles.reverseIntake();
                drive.turn(Math.toRadians(90));
            } else {
                bot.noodles.reverseIntake();
            }
            drive.setPoseEstimate(currentPose);
        }

        telemetry.addData("purple pixel is currently being dropped",".");
        telemetry.update();
    }

    public void stopNoodles(){
        bot.noodles.stop();
        bot.box.resetBox();
        telemetry.addData("noodles are stopped",".");
        telemetry.update();
    }


    private void stageScore(){
        bot.noodles.reverseIntake();
        time.reset();
        while(time.seconds() < 7) {
            bot.box.runWheel(true);
        }
        bot.box.runWheel(false);
        telemetry.addData("Scoring in stage area should occur right now",".");
        telemetry.update();
    }


    private void senseAndScore(){
        //locates and moves to corresponding position on Backdrop based on april tags

        bot.camera.setPipeline(bot.aprilTagsPipeline);
        AprilTagsDetection detection= new AprilTagsDetection();
        currentPose= drive.getPoseEstimate();
        int counter=0;
        detection.detectTag();

        try{
            if(prop== TeamPropDetectionPipeline.TeamProp.ONLEFT){

                //keep strafing left until robot detects AprilTag or if you have run loop over 5 times
                while(detection.getTagOfInterest().id!= 1 && counter<5){
                    detection.detectTag();
                    drive.followTrajectorySequence(strafeLeft);
                    counter++;
                    if(counter==4){
                        throw new AprilTagException("april tag could not be located");
                    }
                }
            }

            else if(prop== TeamPropDetectionPipeline.TeamProp.ONRIGHT){

                while(detection.getTagOfInterest().id!=3 && counter<5){
                    detection.detectTag();
                    drive.followTrajectorySequence(strafeRight);
                    counter++;
                    if(counter==4){
                        throw new AprilTagException("april tag could not be located");
                    }
                }
            }

            else if(prop != TeamPropDetectionPipeline.TeamProp.NOTDETECTED){
                telemetry.addLine("the prop wasn't detected");
            }

            drive.followTrajectorySequence(forward);
            score();
            drive.setPoseEstimate(currentPose);

        }catch(Exception e){
            e.printStackTrace();
        }


        telemetry.addData("Sensing and scoring should occur right now", ".");
        telemetry.update();
    }


    private void score(){
        bot.fourbar.outtake();
        bot.box.depositSecondPixel();
        bot.resetEverything();
        telemetry.addData("Scoring with no sensing should occur right now",".");
        telemetry.update();
    }

   /* private void waitForPartner() throws InterruptedException {
        if(wait){
            wait(15000);
        }
    }

    */

}
