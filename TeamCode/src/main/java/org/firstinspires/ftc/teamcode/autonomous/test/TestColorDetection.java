package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous(name="test teamprop")
public class TestColorDetection extends LinearOpMode {
    OpenCvWebcam camera;
        TeamPropDetectionPipeline pipeline;
    private final int STREAM_HEIGHT = 720, STREAM_WIDTH = 1280;
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize camera and pipeline
        cameraInit();
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Team prop is in the", pipeline.getTeamPropLocation());
            telemetry.update();
        }
    }
    private void cameraInit(){
        WebcamName camName = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        pipeline = new TeamPropDetectionPipeline(telemetry);
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