package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.ExperimentalAuto.Side.BLUE;

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

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

 /*
            SIDE:
                b=red
                a=blue
            DTB:q
                X= Close
                Y= Far
             */


//robot fail has no outtake
//optimal outtakes on the backdrop
// no sense does not involve team prop sensing or april tag sensing (still outtakes)

@Config
@Autonomous(name = "ExperimentalAuto")

public class ExperimentalAuto extends LinearOpMode {

    Bot bot;
    boolean wait = false;
    boolean throughMiddle = false;
    double waitSecondsBeforeStart;

    enum Side {
        RED, BLUE,
    }
    enum DistanceToBackdrop {
        CLOSE, FAR,
    }

    //different paths to follow depending on driver input before match
    enum AutoPath {
        MECHANICAL_FAILURE, OPTIMAL, NO_SENSE, JUST_PARK
    }


    Side side = BLUE;
    DistanceToBackdrop dtb= DistanceToBackdrop.CLOSE;
    AutoPath autopath = AutoPath.OPTIMAL;
    TeamPropDetectionPipeline.TeamProp prop;
    private ElapsedTime time = new ElapsedTime();

    TeamPropDetectionPipeline teamPropDetectionPipeline;


    public static double fx = 1078.03779;
    public static double fy = 1084.50988;
    public static double cx = 580.850545;
    public static double cy = 245.959325;


    Pose2d currentPose;

    // UNITS ARE METERS
    public static double tagSize = 0.032;

    SampleMecanumDrive drive= new SampleMecanumDrive(hardwareMap);

    GamepadEx gp1 = new GamepadEx(gamepad1);


    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    Pose2d startPoseBlueFar = new Pose2d(-36, 52, Math.toRadians(-90));
    Pose2d startPoseBlueClose = new Pose2d(10, 56, Math.toRadians(-90));
    Pose2d startPoseRedClose = new Pose2d(10, -52, Math.toRadians(90));
    Pose2d startPoseRedFar = new Pose2d(-34, -48, Math.toRadians(90));

    Vector2d scoreBackdropBlue = new Vector2d(42, 38);
    Vector2d scoreBackdropRed = new Vector2d(42, -34);


    Vector2d parkingPosBlue = new Vector2d(56, 60);
    Vector2d parkingPosRed = new Vector2d(54, -60);

    TrajectorySequence redFarJustPark = drive.trajectorySequenceBuilder(startPoseRedFar)
            .forward(40)
            .strafeRight(50)
            .lineTo(parkingPosRed)
            .build();

    TrajectorySequence redCloseJustPark = drive.trajectorySequenceBuilder(startPoseRedClose)
            .lineTo(parkingPosRed)
            .build();
    TrajectorySequence blueFarJustPark = drive.trajectorySequenceBuilder(startPoseBlueFar)
            .forward(40)
            .strafeLeft(50)
            .lineTo(parkingPosBlue)
            .build();
    TrajectorySequence blueCloseJustPark = drive.trajectorySequenceBuilder(startPoseBlueClose)
            .lineTo(parkingPosBlue)
            .build();

    TrajectorySequence redFarApproachSpike = drive.trajectorySequenceBuilder(startPoseRedFar)
            .splineTo(new Vector2d(-34, -34), Math.toRadians(90))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, this::facePurplePixel)
            .UNSTABLE_addTemporalMarkerOffset(0, this::goToScore)
            .build();


    TrajectorySequence redCloseApproachSpike = drive.trajectorySequenceBuilder(startPoseRedClose)
            .splineTo(new Vector2d(10, -34), Math.toRadians(90))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, this::facePurplePixel)
            .UNSTABLE_addTemporalMarkerOffset(0, this::goToScore)
            .build();

    TrajectorySequence blueCloseApproachSpike = drive.trajectorySequenceBuilder(startPoseBlueClose)
            .splineTo(new Vector2d(10, 34), Math.toRadians(-90))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, this::facePurplePixel)
            .UNSTABLE_addTemporalMarkerOffset(0, this::goToScore)
            .build();


    TrajectorySequence blueFarApproachSpike = drive.trajectorySequenceBuilder(startPoseBlueFar)
            .splineTo(new Vector2d(-36, 34), Math.toRadians(-90))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, this::facePurplePixel)
            .UNSTABLE_addTemporalMarkerOffset(0, this::goToScore)
            .build();

    TrajectorySequence dropPixelCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .forward(5)
            .UNSTABLE_addTemporalMarkerOffset(-0.3, this::depositPurplePixel)
            .turn(Math.toRadians(-90))
            .build();

    TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .turn(Math.toRadians(-90))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, this::depositPurplePixel)
            .build();

    TrajectorySequence dropPixelRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .turn(Math.toRadians(90))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, this::depositPurplePixel)
            .turn(Math.toRadians(180))
            .build();

    TrajectorySequence redPassCenterTruss = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(new Vector2d(-34, -10))
            .lineTo(new Vector2d(20, -10))
            .build();

    TrajectorySequence bluePassCenterTruss = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(new Vector2d(-36, 10))
            .lineTo(new Vector2d(30, 10))
            .build();

    TrajectorySequence redScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(scoreBackdropRed)
            .UNSTABLE_addTemporalMarkerOffset(0,this::senseAndScore)
            .UNSTABLE_addTemporalMarkerOffset(0,this::park)
            .build();

    TrajectorySequence blueScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(scoreBackdropBlue)
            .UNSTABLE_addTemporalMarkerOffset(0,this::senseAndScore)
            .UNSTABLE_addTemporalMarkerOffset(0,this::park)
            .build();

    TrajectorySequence redScoreNoSense = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(scoreBackdropRed)
            .UNSTABLE_addTemporalMarkerOffset(0,this::score)
            .build();

    TrajectorySequence blueScoreNoSense = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(scoreBackdropBlue)
            .UNSTABLE_addTemporalMarkerOffset(0,this::score)
            .build();

    TrajectorySequence redStageScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(scoreBackdropRed)
            .UNSTABLE_addTemporalMarkerOffset(0,this::stageScore)
            .build();

    TrajectorySequence blueStageScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(scoreBackdropBlue)
            .UNSTABLE_addTemporalMarkerOffset(0,this::stageScore)
            .build();

    TrajectorySequence redPark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(parkingPosRed)
            .build();

    TrajectorySequence bluePark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
            .lineTo(parkingPosBlue)
            .build();

    TrajectorySequence strafeLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeLeft(5)
                .build();

    TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(5)
                .build();


    @Override
    public void runOpMode() throws InterruptedException {

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bot = Bot.getInstance(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry);

        bot.initCamera(teamPropDetectionPipeline);

        prop = teamPropDetectionPipeline.getTeamPropLocation();


        while (!isStarted()) {
            gp1.readButtons();
            drive.updatePoseEstimate();

            checkControls();
            waitForStart();

            if (opModeIsActive() && !isStopRequested()) {

                if(autopath== AutoPath.JUST_PARK){
                    if(side== Side.BLUE){
                        if(dtb== DistanceToBackdrop.CLOSE){
                            drive.followTrajectorySequence(blueCloseJustPark);
                        }
                        else{
                            drive.followTrajectorySequence(blueFarJustPark);
                        }
                    }
                    else{
                        if(dtb== DistanceToBackdrop.CLOSE){
                            drive.followTrajectorySequence(redCloseJustPark);
                        }
                        else{
                            drive.followTrajectorySequence(redFarJustPark);
                        }
                    }
                }
                else {
                    switch (dtb) {
                        case FAR:
                            switch (side) {
                                case BLUE:
                                    drive.followTrajectorySequence(blueFarApproachSpike);
                                    break;
                                case RED:
                                    drive.followTrajectorySequence(redFarApproachSpike);
                                    break;
                            }
                            break;
                        case CLOSE:
                            switch (side) {
                                case BLUE:
                                    drive.followTrajectorySequence(blueCloseApproachSpike);
                                    break;
                                case RED:
                                    drive.followTrajectorySequence(redCloseApproachSpike);
                                    break;
                            }
                            break;
                    }
                }
            }
        }
    }

    private void facePurplePixel(){
        if(autopath==AutoPath.NO_SENSE){
            drive.followTrajectorySequence(dropPixelCenter);
        }
        else {
            if (prop == TeamPropDetectionPipeline.TeamProp.ONLEFT) {
                drive.followTrajectorySequence(dropPixelLeft);

            } else if (prop == TeamPropDetectionPipeline.TeamProp.ONRIGHT) {
                drive.followTrajectorySequence(dropPixelRight);
            } else {
                drive.followTrajectorySequence(dropPixelCenter);
            }
        }
    }

    private void depositPurplePixel(){
        bot.noodles.reverseIntake();
        bot.noodles.stop();
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

    private void goToScore() {

        if(throughMiddle){
            if(side== Side.BLUE){
                drive.followTrajectorySequence(bluePassCenterTruss);
            }
            else{
                drive.followTrajectorySequence(redPassCenterTruss);
            }
        }

        switch (side) {
            case BLUE:
                switch (autopath) {
                    case OPTIMAL:
                        drive.followTrajectorySequence(blueScore);
                        break;
                    case NO_SENSE:
                        drive.followTrajectorySequence(blueScoreNoSense);
                        drive.followTrajectorySequence(redScore);
                        break;
                    case MECHANICAL_FAILURE:
                        drive.followTrajectorySequence(blueStageScore);
                        break;
                }
                break;
            case RED:
                switch (autopath) {
                    case OPTIMAL:
                        drive.followTrajectorySequence(redScore);
                        break;
                    case NO_SENSE:
                        drive.followTrajectorySequence(redScoreNoSense);
                        break;
                    case MECHANICAL_FAILURE:
                        drive.followTrajectorySequence(redStageScore);
                        break;
                }
        }
    }


    private void senseAndScore(){
        AprilTagsPipeline aprilTagsPipeline= new AprilTagsPipeline(tagSize,fx,fy,cx,cy);
        bot.camera.setPipeline(aprilTagsPipeline);
        AprilTagsDetection detection= new AprilTagsDetection();
        int counter=0;
        detection.detectTag();

            if(prop== TeamPropDetectionPipeline.TeamProp.ONLEFT){
                //keep strafing left until robot detects AprilTag or if you have run loop over 5 times
                while(detection.getTagOfInterest().id!= 1 && counter<5){
                    detection.detectTag();
                    drive.followTrajectorySequence(strafeLeft);
                    counter++;
                }
            }

            else if(prop== TeamPropDetectionPipeline.TeamProp.ONRIGHT){
                while(detection.getTagOfInterest().id!=3 && counter<5){
                    detection.detectTag();
                    drive.followTrajectorySequence(strafeRight);
                    counter++;
                }
            }
            score();
    }


    private void score(){
        bot.fourbar.outtake();
        bot.box.depositSecondPixel();
        bot.fourbar.storage();
        telemetry.addData("Scoring with no sensing should occur right now",".");
        telemetry.update();
    }

    private void park(){
        if(side== Side.RED){
            drive.followTrajectorySequence(redPark);
        }
        if(side== Side.BLUE){
            drive.followTrajectorySequence(bluePark);
        }

    }

        private void checkControls(){
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                telemetry.addLine("Alliance: red");
                side = Side.RED;
                teamPropDetectionPipeline.setAlliance(1);
                telemetry.update();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                telemetry.addLine("Alliance: blue");
                side = BLUE;
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
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                telemetry.addLine("Mode: No Sense");
                autopath = AutoPath.NO_SENSE;
                telemetry.update();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                telemetry.addLine("Mode: Optimal");
                autopath = AutoPath.OPTIMAL;
                telemetry.update();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                telemetry.addLine("Mode: Just Park");
                autopath = AutoPath.JUST_PARK;
                telemetry.update();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                telemetry.addLine("wait for 15 seconds before outtaking");
                wait = true;
                telemetry.update();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                telemetry.addLine("pass through center truss");
                throughMiddle = true;
                telemetry.update();
            }
        }
    }
