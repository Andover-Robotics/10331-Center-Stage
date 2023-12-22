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
        Vector2d scoreRed = new Vector2d(42,-34);


        TrajectorySequence redAllianceCloseNoSense = drive.trajectorySequenceBuilder(startPoseRedClose)
                .splineTo(new Vector2d(10,-34), Math.toRadians(90))
               // .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
               // .UNSTABLE_addTemporalMarkerOffset(0,this::stopNoodles)
                .splineTo(scoreRed, Math.toRadians(0))
               // .UNSTABLE_addTemporalMarkerOffset(-0.5, this::score)
                .waitSeconds(1)
                .lineTo(parkingPosRed)
                .build();

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(startPoseRedClose);
            drive.followTrajectorySequence(redAllianceCloseNoSense);
        }


    }
}
