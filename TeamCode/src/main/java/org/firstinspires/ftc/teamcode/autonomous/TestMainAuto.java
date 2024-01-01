package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "TestMainAutonomous")

public class TestMainAuto extends LinearOpMode {

    Bot bot;
    Pose2d currentPose;

   // TeamPropDetectionPipeline.TeamProp prop;
    private ElapsedTime time = new ElapsedTime();


    public static double fx = 1078.03779;
    public static double fy = 1084.50988;
    public static double cx = 580.850545;
    public static double cy = 245.959325;

    TrajectorySequence strafeLeft;
    TrajectorySequence strafeRight;
    TrajectorySequence forward;

  //  TeamPropDetectionPipeline.TeamProp prop;
    OpenCvWebcam camera;
    TeamPropDetectionPipeline pipeline;

    // UNITS ARE METERS
    public static double tagSize = 0.032;

    SampleMecanumDrive drive;
    private final int STREAM_HEIGHT = 720, STREAM_WIDTH = 1280;
    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName camName = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        pipeline = new TeamPropDetectionPipeline(telemetry);
        camera.setPipeline(pipeline);
        pipeline.setAlliance(2);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", errorCode);
                telemetry.update();
            }
        });


        bot = Bot.getInstance(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Pose2d startPoseBlueFar = new Pose2d(-36, 52, Math.toRadians(-90));
        Pose2d startPoseBlueClose = new Pose2d(10, 56, Math.toRadians(-90));
        Pose2d startPoseRedClose = new Pose2d(10, -52, Math.toRadians(90));
        Pose2d startPoseRedFar = new Pose2d(-34, -48, Math.toRadians(90));



        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        //writing variables for Vector2d positions that are reused
        Vector2d parkingPosBlue = new Vector2d(56,56);
        Vector2d parkingPosRed = new Vector2d(56,-56);

        Vector2d scoreBlue = new Vector2d(42,38);
        Vector2d scoreRed = new Vector2d(40,-25);


        TrajectorySequence redAllianceCloseNoSense = drive.trajectorySequenceBuilder(startPoseRedClose)
                .splineTo(new Vector2d(10,-17 ), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-1, this::dropPurplePixel)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .back(7)
                .splineTo(scoreRed, Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, this::score)
                .waitSeconds(2)
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(1,this::reset)
                .waitSeconds(5)
                .back(5)
                //fix with MeepMeep
                .splineTo(parkingPosRed, Math.toRadians(90))
                .build();

        TrajectorySequence redAllianceCloseRobotFail = drive.trajectorySequenceBuilder(startPoseRedClose)
                .forward(17)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(-90))
                .forward(13)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .back(8)
                .strafeRight(30)
                .forward(15)
                .build();

        TrajectorySequence blueAllianceCloseRobotFail = drive.trajectorySequenceBuilder(startPoseBlueClose)
                .forward(17)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(90))
                .forward(13)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .back(8)
                .strafeLeft(30)
                .forward(15)
                .build();

        TrajectorySequence blueAllianceCloseScore = drive.trajectorySequenceBuilder(startPoseBlueClose)
                .forward(17)
                .UNSTABLE_addTemporalMarkerOffset(0,this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .turn(Math.toRadians(90))
                .forward(13)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,this::stageScore)
                .waitSeconds(12)
                .UNSTABLE_addTemporalMarkerOffset(-0.1,this::reset)
                .waitSeconds(3)
              //  .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .back(10)
                .strafeLeft(30)
                .forward(15)
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

        waitForStart();


        if (opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(startPoseBlueClose);
            drive.followTrajectorySequence(blueAllianceCloseScore);
        }


    }



    private void dropPurplePixel(){
            currentPose = drive.getPoseEstimate();
            if (pipeline.getTeamPropLocation() == TeamPropDetectionPipeline.TeamProp.ONLEFT) {
                drive.turn(Math.toRadians(90));
                bot.noodles.reverseIntake();
                drive.turn(Math.toRadians(-90));
            } else if (pipeline.getTeamPropLocation() == TeamPropDetectionPipeline.TeamProp.ONRIGHT) {
                drive.turn(Math.toRadians(-90));
                bot.noodles.reverseIntake();
                drive.turn(Math.toRadians(90));
            } else {
                bot.noodles.reverseIntake();
            }
            drive.setPoseEstimate(currentPose);
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
        while(time.seconds() < 4) {
            bot.box.runWheel(true);
            //need to replace this servo
        }
        bot.box.runWheel(true);
        telemetry.addData("Scoring in stage area should occur right now",".");
        telemetry.update();
    }


    private void senseAndScore(){
        AprilTagsPipeline aprilTagsPipeline= new AprilTagsPipeline(tagSize,fx,fy,cx,cy);
        bot.camera.setPipeline(aprilTagsPipeline);
        AprilTagsDetection detection= new AprilTagsDetection();
        currentPose= drive.getPoseEstimate();
        int counter=0;
        detection.detectTag();


        //keep strafing left until robot detects AprilTag or if you have run loop over 5 times
        while(detection.getTagOfInterest().id!= 1 && counter<5){
            detection.detectTag();
            drive.followTrajectorySequence(strafeLeft);
            counter++;
        }


        drive.followTrajectorySequence(forward);
        score();
        drive.setPoseEstimate(currentPose);

    }




    private void score(){
        bot.fourbar.outtake();
        bot.box.depositSecondPixel();
        telemetry.addData("Scoring with no sensing should occur right now",".");
        telemetry.update();
    }
    private void reset(){
        bot.box.resetBox();
        bot.fourbar.storage();
        telemetry.addData("Scoring with no sensing should occur right now",".");
        telemetry.update();
    }
}
