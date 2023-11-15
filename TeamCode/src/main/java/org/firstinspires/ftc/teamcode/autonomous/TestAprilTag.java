package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "MainAutonomous")



public class TestAprilTag extends LinearOpMode {

    Bot bot;

    private final double fx = 1078.03779;
    private final double fy = 1084.50988;
    private final double cx = 580.850545;
    private final double cy = 245.959325;

    // UNITS ARE METERS
    //might have to change this? not sure what size the tags will be
    private final double tagsize = 0.166;

    static int ONE = 1;
    static int TWO = 2;
    static int THREE = 3;


    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        bot.camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        bot.aprilTagsPipeline= new AprilTagsPipeline(tagsize, fx, fy, cx, cy);
        ;
        bot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                bot.camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        AprilTagsDetection.detectTag();

        telemetry.addData("The April Tag is ", AprilTagsDetection.getTag());
        telemetry.update();

    }
}
