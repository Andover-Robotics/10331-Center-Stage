package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
test sequential teleop, fine tune if needed (it will be needed)
 */

@TeleOp
public class SequentialTest extends LinearOpMode {
    private double driveSpeed;
    private GamepadEx gp1, gp2;
    private ElapsedTime time;
    Bot bot;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        time = new ElapsedTime();
        telemetry.addData("boxAnglePosition:", bot.fourbar.getBoxStoragePos());

        bot.reverseMotors();
        bot.slides.resetEncoder();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            gp2.readButtons();
            gp1.readButtons();

            telemetry.addData("Sequential TeleOp has started","wheeeee");
            drive();

            if(gp2.wasJustPressed(GamepadKeys.Button.A))
                intake();


            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                outtake(1);
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                outtake(2);
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                outtake(3);
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                outtake(4);

        }


    }
    private void drive() {
        driveSpeed = 1;

        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);

        bot.driveRobotCentric(-driveVector.getX() * driveSpeed,
                -driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed / 1.7
        );
    }

    private void intake() {
        time.reset();
        do { bot.noodles.intake(); }
        while(time.seconds() <= 3);
    }

    private void outtake(int stage) {
        bot.fourbar.outtake();
        //because we dont have run to based on position and only based on power,
        //gonna cope and run the manual runTo for a set amount of seconds until it gets to the desired pos
        //testing is required to see how many seconds to run it for before it gets to each stage

        switch(stage) {
            case 1:
                time.reset();
                while(time.seconds() <= 2) {
                    bot.slides.runToManual(0.8);
                }
                bot.slides.brake();
                break;
            case 2:
                time.reset();
                while(time.seconds() <= 3) {
                    bot.slides.runToManual(0.8);
                }
                bot.slides.brake();
                break;
            case 3:
                time.reset();
                while(time.seconds() <= 4) {
                    bot.slides.runToManual(0.8);
                }
                bot.slides.brake();
                break;
            case 4:
                time.reset();
                while(time.seconds() <= 5) {
                    bot.slides.runToManual(0.8);
                }
                bot.slides.brake();
                break;
        }

    }
}
