package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.TeamPropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class MainAutoTwo extends LinearOpMode {

    enum Side {
        RED, BLUE,
    }
    enum DistanceToBackdrop {
        CLOSE, FAR,
    }

    Side side = Side.RED;
    DistanceToBackdrop dtb= DistanceToBackdrop.CLOSE;


    Bot bot;
    private GamepadEx gp1;
    private GamepadEx gp2;
    private double driveSpeed = 0.5;
    private double driveTime = 0.5; // in seconds
    Vector2d driveVector;
    private boolean justPark;


    private ElapsedTime time = new ElapsedTime();
    TeamPropDetectionPipeline.TeamProp prop;

    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        WebcamName camName = hardwareMap.get(WebcamName.class, "webcam");
        bot.camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        TeamPropDetectionPipeline teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry);


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


        // Initializing the robot
        bot.noodles.storage();
        bot.fixMotors();

        while(!opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

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

            if(gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                driveTime-=0.05;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                driveTime+=0.05;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.X)) {
                driveSpeed-=0.1;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.B)) {
                driveSpeed+=0.1;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                justPark = true;
            }

            prop = teamPropDetectionPipeline.getTeamPropLocation();
            telemetry.addData("alliance is ", side);
            telemetry.addData("distance to backdrop is ", dtb);
            telemetry.addData("detected prop", prop.toString());
            telemetry.addData("drive seconds", driveTime);
            telemetry.addData("drive speed", driveSpeed);
            telemetry.addData("just park:", justPark);
            telemetry.update();
        }

        waitForStart();

        telemetry.addLine("Autonomous has started");
        time.reset();

        if(justPark) {
            if (side == Side.RED && dtb == DistanceToBackdrop.CLOSE)
                back();
            else if (side == Side.BLUE && dtb == DistanceToBackdrop.CLOSE)
                back();
            else if (side == Side.BLUE && dtb == DistanceToBackdrop.FAR) {
                back();
                right();
            }
            else if (side == Side.RED && dtb == DistanceToBackdrop.FAR)
                back();

            driveSpeed = 0.8;
            driveTime = 5;
            park();
        } else {
            moveBasedOnProp();

            if(side==Side.RED && dtb==DistanceToBackdrop.CLOSE) {
                //RED CLOSE
                driveSpeed = 0.7;
                while (time.seconds() < driveTime) {
                    right();
                }
                depositPixel();
                driveSpeed = 0.2;
                while (time.seconds() < driveTime + 1.6) {
                    back();
                }
                driveSpeed = 0.05;
         /*   while (time.seconds() < driveTime + 1.5 + 0.1 + 0.2) {
                right();
            }

          */
            }

           /*
Red close
Left: nothing; right back
Center: back ;right back
Right: backer; right back
            */
            if(side==Side.BLUE && dtb==DistanceToBackdrop.CLOSE) {
                //blue close

                driveSpeed = 0.7;
                while (time.seconds() < driveTime) {
                    left();
                }
                depositPixel();
                driveSpeed = 0.2;
                while (time.seconds() < driveTime + 1.5 + 0.1) {
                    back();
                }
            }

        /*
        Blue close
Left: backer ; left back
Center: back ; left back
Right: nothing; left back
         */

            if(side==Side.RED && dtb==DistanceToBackdrop.FAR) {
                //red far

                driveSpeed = 0.7;
                while (time.seconds() < driveTime) {
                    left();
                }
                depositPixel();
                driveSpeed = 0.2;
                while (time.seconds() < driveTime + 1.5 + 0.1) {
                    lilback();
                }
            }
/*
        Red far
        Left: backer ; left lil back
        Center: back;  left lil back
        Right: nothing ; left lil back
        AFTER PLACED:
        Left forward right?

 */
            if(side==Side.BLUE && dtb==DistanceToBackdrop.FAR) {
                //blue far

                driveSpeed = 0.7;
                while (time.seconds() < driveTime) {
                    lilback();
                }
                depositPixel();
            }
/*
        Blue far
        Left:  right; lil back
        Center: back right; lil back
        Right: back; lil back
        AFTER PLACED:
        Right forward left?
     */
            sleep(2500);

        }

    }

    private void drive(){
        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed, 0
        );
    }

    private void forward(){
        driveVector= new Vector2d(0.0, -1.0);
        drive();
    }
    private void back(){
        driveVector= new Vector2d(0.0, 1.0);
        drive();
    }
    private void backer(){
        driveVector= new Vector2d(0.0, 1.5);
        drive();
    }
    private void lilback(){
        driveVector= new Vector2d(0.0, 0.5);
        drive();
    }
    private void right(){
        driveVector= new Vector2d(1.0, 0);
        drive();
    }
    private void left(){
        driveVector= new Vector2d(-1, 0);
        drive();
    }

    private void depositPixel(){
        bot.noodles.reverseIntake(0.1);
        sleep(1500);
        bot.noodles.storage();
    }

    private void park() {
        switch(side) {
            case RED:
                while(time.seconds() < driveTime)
                    right();
                    backer();

            case BLUE:
                while(time.seconds() < driveTime)
                    left();
                    backer();
        }
    }

    private void moveBasedOnProp(){
        if(prop == TeamPropDetectionPipeline.TeamProp.ONLEFT){
            if(side==Side.BLUE && dtb== DistanceToBackdrop.CLOSE)
                backer();
            else if(side==Side.BLUE && dtb== DistanceToBackdrop.FAR)
                right();
            else if(side==Side.RED && dtb== DistanceToBackdrop.FAR)
                backer();
        }

        if(prop == TeamPropDetectionPipeline.TeamProp.MIDDLE) {
            if (side == Side.RED && dtb == DistanceToBackdrop.CLOSE)
                back();
            else if (side == Side.BLUE && dtb == DistanceToBackdrop.CLOSE)
                back();
            else if (side == Side.BLUE && dtb == DistanceToBackdrop.FAR) {
                back();
                right();
            }
            else if (side == Side.RED && dtb == DistanceToBackdrop.FAR)
                back();
        }

        if(prop == TeamPropDetectionPipeline.TeamProp.ONRIGHT) {
            if (side == Side.RED && dtb == DistanceToBackdrop.CLOSE)
                backer();
            else if (side == Side.BLUE && dtb == DistanceToBackdrop.FAR)
                back();
        }
    }
    /*
Red close
Left: nothing; right back
Center: back ;right back
Right: backer; right back

Blue close
Left: backer ; left back
Center: back ; left back
Right: nothing; left back

Red far
Left: backer ; left lil back
Center: back;  left lil back
Right: nothing ; left lil back
AFTER PLACED:
Left forward right?

Blue far
Left:  right; lil back
Center: back right; lil back
Right: back; lil back
AFTER PLACED:
Right forward left?
     */


}