package org.firstinspires.ftc.teamcode.autonomous.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */

/*
LRR guidelines:
    Run the BackAndForth opmode via the RC.
    Then, connect to the RC phone's wifi network.
    Navigate to RC or 192.168.43.1:8080/dash with a Control Hub.
    Ensure that you have the Field view selected in the top right.
    You should see two lines and two circles being drawn: green for the target position
    and blue for your bot's actual position.

    Look for the SampleMecanumDrive in the right sidebar. Open that dropdown.
    You should be seeing two options: HEADING_PID and TRANSLATION_PID.
    Both options are located in the SampleMecanumDrive file.

    Open up HEADING_PID first.
    Just keep increasing kP until the robot starts to keep an accurate heading.
    This was around 8 in my experience, although your mileage may vary. You should not need to adjust kD and kI.

    Open up TRANSLATION_PID next. Once again, keep increasing kP until the robot starts to adjust itself and follows the path.
    This was also around 8 in my experience, although your mileage may vary. You should not need to adjust kD and kI.

    Once that's tuned, you should be done! Remember that any changes in Dashboard must be reflected in the appropriate file.
    So, you should copy-paste your numbers into the PID object in SampleMecanumDrive.java.
    Feel free to run the same tuning process with FollowerPIDTuner. This is encouraged for further accuracy.

    You should be done! Go on to the SplineTest to ensure that your following is accurate.
 */

@Config
@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    public static double DISTANCE = 48; // in
    //can make this smaller if we dont have space ;-;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }
    }
}