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
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


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


@Config
@Autonomous(name = "MainAutonomous")

public class ExperimentalAuto extends LinearOpMode {

    Bot bot;
    boolean wait = false;
    boolean throughMiddle = true;
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


        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Pose2d startPoseBlueFar = new Pose2d(-36, 52, Math.toRadians(-90));
        Pose2d startPoseBlueClose = new Pose2d(10, 56, Math.toRadians(-90));
        Pose2d startPoseRedClose = new Pose2d(10, -52, Math.toRadians(90));
        Pose2d startPoseRedFar = new Pose2d(-34, -48, Math.toRadians(90));

        Vector2d purplePixelRightPosBlueClose = new Vector2d(56, 56);
        Vector2d purplePixelLeftPosBlueClose = new Vector2d(56, 56);
        Vector2d purplePixelCenterPosBlueClose = new Vector2d(56, 56);

        Vector2d purplePixelRightPosBlueFar = new Vector2d(56, 56);
        Vector2d purplePixelLeftPosBlueFar = new Vector2d(56, 56);
        Vector2d purplePixelCenterPosBlueFar = new Vector2d(56, 56);

        Vector2d purplePixelRightPosRedFar = new Vector2d(56, 56);
        Vector2d purplePixelLeftPosRedFar = new Vector2d(56, 56);
        Vector2d purplePixelCenterPosRedFar = new Vector2d(56, 56);

        Vector2d purplePixelRightPosRedClose = new Vector2d(56, 56);
        Vector2d purplePixelLeftPosRedClose = new Vector2d(56, 56);
        Vector2d purplePixelCenterPosRedClose = new Vector2d(56, 56);

        Vector2d scoreBackdropBlue = new Vector2d(42, 38);
        Vector2d scoreBackdropRed = new Vector2d(42, -34);


        Vector2d parkingPosBlue = new Vector2d(56, 56);
        Vector2d parkingPosRed = new Vector2d(56, -56);


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

        prop = teamPropDetectionPipeline.getTeamPropLocation();

        TrajectorySequence redAllianceFar = drive.trajectorySequenceBuilder(startPoseRedFar)
                .splineTo(new Vector2d(-34, -34), Math.toRadians(90))
                .build();


        TrajectorySequence redAllianceClose = drive.trajectorySequenceBuilder(startPoseRedClose)
                .splineTo(new Vector2d(10, -34), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .build();

        TrajectorySequence dropPixelCenter = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(5)
                .build();

        TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence dropPixelRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence redAllianceScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(scoreBackdropRed)
                .build();

        TrajectorySequence redAlliancePark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(parkingPosRed)
                .build();


        TrajectorySequence blueAllianceCloseNoSense = drive.trajectorySequenceBuilder(startPoseBlueClose)
                .splineTo(new Vector2d(10, 38), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0, this::stopNoodles)
                .splineTo(scoreBackdropBlue, Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, this::score)
                .waitSeconds(1)
                .lineTo(parkingPosBlue)
                .build();

        TrajectorySequence blueAllianceFarNoSense = drive.trajectorySequenceBuilder(startPoseBlueFar)
                .splineTo(new Vector2d(-36, 34), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, this::dropPurplePixel)
                .waitSeconds(1.5)
                .forward(20)
                .lineTo(new Vector2d(-16, 10))
                .UNSTABLE_addTemporalMarkerOffset(0, this::stopNoodles)
                .strafeLeft(30)
                .splineTo(scoreBackdropBlue, Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, this::score)
                .waitSeconds(1)
                .lineTo(parkingPosBlue)
                .build();



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
                telemetry.addLine("not through middle");
                throughMiddle = false;
                telemetry.update();
            }
            while (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                if (waitSecondsBeforeStart >= 10) break;
                waitSecondsBeforeStart++;
            }


        }
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            drive.followTrajectorySequence(blueAllianceCloseNoSense);
        }
    }

    private void dropPurplePixel(){

        //  DIRECT BOT TO VARIOUS TRAJECTORIES BASED ON SPIKE MARK LOCATION

        if (prop == TeamPropDetectionPipeline.TeamProp.ONLEFT) {

        } else if (prop == TeamPropDetectionPipeline.TeamProp.ONRIGHT) {


        } else {

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
        while(time.seconds() < 7) {
            bot.box.runWheel(true);
        }
        bot.box.runWheel(false);
        telemetry.addData("Scoring in stage area should occur right now",".");
        telemetry.update();
    }


    private void senseAndScore(){
        //locates and moves to corresponding position on Backdrop based on april tags
        //Note: can change this based on the backdropAlign method if it works

        AprilTagsPipeline aprilTagsPipeline= new AprilTagsPipeline(tagSize,fx,fy,cx,cy);
        bot.camera.setPipeline(aprilTagsPipeline);
        AprilTagsDetection detection= new AprilTagsDetection();
        int counter=0;
        detection.detectTag();

        //** hack is also found in advanced tips of rr page (teleop rr)

        // WONDERFUL HACK: CHECK SCREENSHOT ON HOW TO UPDATE LOCALIZER POSITION WHEN STRAFING
        // SAVE CURRENT LOCALIZER POSITION (BASED ON ENDING OF PREVIOUS TRAJECTORY)
        // STRAFE ROBOT UNTIL YOU DETECT APRIL TAG
        // THEN UPDATE LOCALIZER POSITION
        // FOLLOWING TRAJECTORY (PARK) SHOULD GET STARTPOS DIRECTLY FROM LOCALIZER

    }


    private void score(){
        bot.fourbar.outtake();
        bot.box.depositSecondPixel();
        bot.resetEverything();
        telemetry.addData("Scoring with no sensing should occur right now",".");
        telemetry.update();
    }


}
