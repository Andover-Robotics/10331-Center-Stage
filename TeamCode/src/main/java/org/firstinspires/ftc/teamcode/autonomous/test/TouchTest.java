package org.firstinspires.ftc.teamcode.autonomous.test;

/// configuration details: https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/configuring_digital_touch_sensor/configuring-digital-touch-sensor.html

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;


@TeleOp
public class TouchTest extends LinearOpMode {
    TouchSensor touch;
    SampleMecanumDrive drive;
    Pose2d startPose = new Pose2d(0,0,0);


    TrajectorySequence forward = drive.trajectorySequenceBuilder(startPose)
            .forward(2)
            .build();

    @Override
    public void runOpMode() {
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        drive= new SampleMecanumDrive(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            while(!touch.isPressed())
                drive.followTrajectorySequence(forward);
        }
    }
}