package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//Important note: When running this opmode, the bot should already be close to the backboard (not too close)
//experiment with changing optimalDistanceFromBackdrop
@TeleOp(group="test")
public class DriveTuningTester extends LinearOpMode {
    BotTest bot;
    GamepadEx gp1;
    private double driveSpeed =1;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = BotTest.getInstance(this);
        gp1 = new GamepadEx(gamepad1);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            //drive(); add this after april tag tuning method is known to work
            bot.aprilTagTuning();

        }
    }

    /*private void drive() {
        gp1.readButtons();

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

     */
}
