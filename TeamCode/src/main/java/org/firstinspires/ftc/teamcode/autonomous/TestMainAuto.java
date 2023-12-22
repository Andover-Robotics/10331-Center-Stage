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

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "TestMainAutonomous")

public class TestMainAuto extends LinearOpMode {

    Bot bot;

   // TeamPropDetectionPipeline.TeamProp prop;
    private ElapsedTime time = new ElapsedTime();


    public static double fx = 1078.03779;
    public static double fy = 1084.50988;
    public static double cx = 580.850545;
    public static double cy = 245.959325;


    // UNITS ARE METERS
    public static double tagSize = 0.032;

    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {

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

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(startPoseRedClose);
            drive.followTrajectorySequence(redAllianceCloseRobotFail);
        }


    }



    private void dropPurplePixel(){
       // if(autopath== MainAuto.AutoPath.NO_SENSE){
            bot.noodles.reverseIntake();
       // }
       /* else {
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

        */
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


   /* private void senseAndScore(){
        //locates and moves to corresponding position on Backdrop based on april tags

        AprilTagsPipeline aprilTagsPipeline= new AprilTagsPipeline(tagSize,fx,fy,cx,cy);
        bot.camera.setPipeline(aprilTagsPipeline);
        AprilTagsDetection detection= new AprilTagsDetection();
        int counter=0;
        detection.detectTag();

            drive.followTrajectorySequence(forward);
            score();
            drive.setPoseEstimate(currentPose);

        }catch(Exception e){
            e.printStackTrace();
        }


        telemetry.addData("Sensing and scoring should occur right now", ".");
        telemetry.update();
    }

    */


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
