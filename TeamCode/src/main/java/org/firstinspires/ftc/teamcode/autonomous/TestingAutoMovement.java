package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gp1 = new GamepadEx(gamepad1);

        drive= new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        strafeLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeLeft(20)
                .build();
        strafeRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(20)
                .build();
        forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(20)
                .build();
        backward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(20)
                .build();


        while (!isStarted()) {

            gp1.readButtons();

            waitForStart();

            if (opModeIsActive() && !isStopRequested()) {
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    drive.followTrajectorySequence(strafeLeft);
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    drive.followTrajectorySequence(strafeRight);
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    drive.followTrajectorySequence(forward);
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    drive.followTrajectorySequence(backward);
                }

            }
        }
    }

}