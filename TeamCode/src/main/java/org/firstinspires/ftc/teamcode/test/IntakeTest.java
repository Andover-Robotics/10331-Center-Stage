package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;

@TeleOp
public class IntakeTest extends LinearOpMode {
    Bot bot;
    private GamepadEx gp2;

    @Override
    public void runOpMode() {
        bot = Bot.getInstance(this);
        gp2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            gp2.readButtons();
            telemetry.update();

            if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                //counteroller moved instead
                bot.noodles.incrementServoLeft();
                telemetry.addLine(" position is " + bot.noodles.extendTwo.getPosition());
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                bot.noodles.decrementServoLeft();
                telemetry.addLine("position " +  bot.noodles.extendTwo.getPosition());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                bot.noodles.incrementServoRight();
                telemetry.addLine("Box position is " +  bot.noodles.extendOne.getPosition());
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.noodles.decrementServoRight();
                telemetry.addLine("Box position is " + bot.noodles.extendOne.getPosition());
            }


            telemetry.update();

        }
    }
}
