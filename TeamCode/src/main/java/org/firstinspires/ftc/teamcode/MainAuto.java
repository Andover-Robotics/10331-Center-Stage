package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class MainAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;
    private GamepadEx gp2;
    private double driveSpeed = 0.5;
    private double driveTime = 0.5; // in seconds
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        // Initializing the robot
        bot.noodles.goToIntakePos();
        bot.reverseMotors();
        bot.slides.resetEncoder();
        bot.slides.resetProfiler();
        bot.slides.runTo(0);
        bot.noodles.storage();
        bot.fixMotors();

        while(!opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
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
            telemetry.addData("drive seconds", driveTime);
            telemetry.addData("drive speed", driveSpeed);
            telemetry.update();
        }

        waitForStart();

        telemetry.addLine("Autonomous has started");
        time.reset();

        // drive somewhere
        // calculate drive vector
        //RED CLOSE
        driveSpeed = 0.7;
        Vector2d driveVector = new Vector2d(1.0, 0.0); // x - strafe, y - forward/backward
        // run drive function
        while(time.seconds() < driveTime) {
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed, 0
            );
        }
        bot.driveRobotCentric(0, 0, 0);
        // Reverse intake
        bot.noodles.goToIntakePos();
        sleep(1500);
        bot.noodles.storage();
        driveSpeed = 0.2;
        driveVector = new Vector2d(0.0, 1.0);
        // run drive function
        while(time.seconds() < driveTime + 1.5 + 0.1) {
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed, 0
            );
        }
        driveSpeed = 0.05;
        driveVector = new Vector2d(1.0, 0.0);
        // run drive function
        while(time.seconds() < driveTime + 1.5 + 0.1 + 0.2) {
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed, 0
            );
        }

        sleep(2500);
        //requestOpModeStop();
    }
}