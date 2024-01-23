package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;

@TeleOp(group = "test")
public class DriveTest extends LinearOpMode{

    private GamepadEx gp1;
    Bot bot;
    private double driveSpeed=1;
    private boolean isFieldCentric;

    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp1.readButtons();

        waitForStart();
        bot.reverseMotors();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("TeleOp has started","wheeeee");

            if(isFieldCentric){
                driveRobotCentric();
            }
            else{
                driveFieldCentric();
            }

            if(gp1.wasJustPressed(GamepadKeys.Button.START)){
                isFieldCentric=!isFieldCentric;
            }
        }

    }

    private void driveRobotCentric() {
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

    private void driveFieldCentric() {
            driveSpeed = 1 - 0.8 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            driveSpeed = Math.max(0, driveSpeed);
            bot.fixMotors();

            Vector2d driveVector = new Vector2d(-gp1.getLeftX(), -gp1.getLeftY()),
                    turnVector = new Vector2d(-gp1.getRightX(), 0);

            bot.driveFieldCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
    }

}