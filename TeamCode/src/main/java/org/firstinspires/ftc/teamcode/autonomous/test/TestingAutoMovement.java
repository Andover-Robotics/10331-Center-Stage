package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;




//robot fail has no outtake
//optimal outtakes on the backdrop
// no sense does not involve team prop sensing or april tag sensing (still outtakes)

@Config
@Autonomous
public class TestingAutoMovement extends LinearOpMode{

    SampleMecanumDrive drive;

    TrajectorySequence strafeLeft;
    TrajectorySequence strafeRight;
    TrajectorySequence forward;
    TrajectorySequence backward;

    Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));

    enum AutoPath {
       FOWARD, BACKWARD, STRAFE_RIGHT, STRAFE_LEFT
    }

    AutoPath autopath;



    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gp1 = new GamepadEx(gamepad1);

        drive= new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        strafeLeft = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(10)
                .build();
        strafeRight = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(10)
                .build();
        forward = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .build();
        backward = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .build();


        while (!isStarted())

            gp1.readButtons();

            while(!gp1.wasJustPressed(GamepadKeys.Button.START)) {
                gp1.readButtons();
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    autopath = AutoPath.STRAFE_LEFT;
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    autopath = AutoPath.STRAFE_RIGHT;
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    autopath = AutoPath.FOWARD;
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    autopath = AutoPath.BACKWARD;
                }
            }

            waitForStart();

            if (opModeIsActive() && !isStopRequested()) {
                gp1.readButtons();
                if (autopath==AutoPath.STRAFE_LEFT) {
                    drive.followTrajectorySequence(strafeLeft);
                }
                if (autopath==AutoPath.STRAFE_RIGHT) {
                    drive.followTrajectorySequence(strafeRight);
                }
                if (autopath==AutoPath.FOWARD){
                    drive.followTrajectorySequence(forward);
                }
                if (autopath==AutoPath.BACKWARD) {
                    drive.followTrajectorySequence(backward);
                }

            }
        }
    }
