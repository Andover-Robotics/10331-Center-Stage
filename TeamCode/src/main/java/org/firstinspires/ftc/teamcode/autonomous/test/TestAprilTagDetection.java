package org.firstinspires.ftc.teamcode.autonomous.test;

import static org.firstinspires.ftc.teamcode.autonomous.MainAuto.cx;
import static org.firstinspires.ftc.teamcode.autonomous.MainAuto.cy;
import static org.firstinspires.ftc.teamcode.autonomous.MainAuto.fx;
import static org.firstinspires.ftc.teamcode.autonomous.MainAuto.fy;
import static org.firstinspires.ftc.teamcode.autonomous.MainAuto.tagSize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AprilTagsDetection;
import org.firstinspires.ftc.teamcode.autonomous.AprilTagsPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class TestAprilTagDetection extends LinearOpMode {
    OpenCvWebcam camera;
    AprilTagsPipeline pipeline;
    AprilTagsDetection detection = new AprilTagsDetection();
    private final int STREAM_HEIGHT = 720, STREAM_WIDTH = 1280;
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize camera and pipeline

        cameraInit();
        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            detection.detectTag();
            telemetry.addLine("The tag detected is:" + detection.getTagOfInterest());
            telemetry.addLine("the distance to tag is " + detection.calcDistToTag());
            telemetry.update();
        }

    }
    private void cameraInit(){
        WebcamName camName = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        pipeline = new AprilTagsPipeline(tagSize,fx,fy,cx,cy);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", errorCode);
                telemetry.update();
            }
        });
    }
}