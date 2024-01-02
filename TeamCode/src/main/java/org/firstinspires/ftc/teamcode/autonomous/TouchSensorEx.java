package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

@Autonomous(name="Touch Sensor Test")
public class TouchSensorEx extends LinearOpMode {
    Bot bot;
    TouchSensor sensor;
    SampleMecanumDrive drive;
    Pose2d startPose = new Pose2d(0,0,0);
    double distanceTravelled = 0;
    boolean wasTouched = true;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        sensor = hardwareMap.touchSensor.get("touchSensor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TrajectorySequence goatedTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(10)
                .build();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            while(!sensor.isPressed()) {
                drive.followTrajectorySequence(goatedTraj);
                distanceTravelled += 10;
                telemetry.addData("Sensor not touched, distance travelled is ",distanceTravelled);
                telemetry.update();
                if(distanceTravelled > 100) {
                    wasTouched = false;
                    break;
                }
            }

            if(!wasTouched) telemetry.addData("Sensor not touched, troubleshoot pls","");
            else telemetry.addData("Sensor was touched ;)","");
            telemetry.update();
        }
    }


    public boolean detectTouch(TouchSensor sensor) {
        if(sensor.isPressed()) return false;
        return true;
    }
}
