package org.firstinspires.ftc.teamcode.autonomous.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.autonomous.DriveConstants;
import org.firstinspires.ftc.teamcode.autonomous.SampleMecanumDrive;


//TUNE LATERAL DISTANCE FIRST!
//TUNE MAX ANGULAR VELOCITY TUNER FIRST (it's quick trust)

/*

Track width = the center-to-center distance from two parallel wheels.
However, there are two types of track width that you may see: track width for drivetrain and for deadwheel odo
Drivetrain trackwidth is used for the forward kinematics for feedforward following.
The track width for three-wheel odometry (aka LATERAL_DISTANCE) is used for localization.
You should have already tuned localization. Right now, you will be tuning the drive train.
 */

/*
 * This routine determines the effective track width. The procedure works by executing a point turn
 * with a given angle and measuring the difference between that angle and the actual angle (as
 * indicated by an external IMU/gyro, track wheels, or some other localizer). The quotient
 * given angle / actual angle gives a multiplicative adjustment to the estimated track width
 * (effective track width = estimated track width * given angle / actual angle). The routine repeats
 * this procedure a few times and averages the values for additional accuracy. Note: a relatively
 * accurate track width estimate is important or else the angular constraints will be thrown off.
 */

/*
TLDR:
Run TrackWidthTuner opMode. Bot should turn 180 degrees 5 times
Do nOT TOUCH THE BOT DURING TUNING PROCESS
At the end of tuning, RC telemetry should print "effective track width"
If the number is close to the actual track width, stick this number into DriveConstants file under TRACK_WIDTH
    if FTC dashboard is open, you can change values and rerun (makes it so that you don't have to repush everytime)
If you bot runs into the following issues, you will have to tune manually:
    It does not turn 180 degrees each time, even with tuning
    The effective track width given does not print something reasonable (most likely a low number like 3)
        ^usually happens because we didn't put initial estimate in drive constants file.
To tune the track width manually, simply keep adjusting the track width yourself until it turns 180 degrees.
If the the bot turns less than 180 degrees, raise the trackwidth.
If the bot turns more than 180 degrees, lower the trackwidth.
 */

@Config
@Autonomous(group = "drive")
public class TrackWidthTuner extends LinearOpMode {
    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // TODO: if you haven't already, set the localizer to something that doesn't depend on
        // drive encoders for computing the heading

        telemetry.addLine("Press play to begin the track width tuner routine");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            drive.turnAsync(Math.toRadians(ANGLE));

            while (!isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.normDelta(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double trackWidth = DriveConstants.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
            trackWidthStats.add(trackWidth);

            sleep(DELAY);
        }

        telemetry.clearAll();
        telemetry.addLine("Tuning complete");
        telemetry.addLine(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(),
                trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}