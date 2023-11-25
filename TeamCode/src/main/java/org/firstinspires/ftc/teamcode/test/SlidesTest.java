package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;

@TeleOp(group = "test")
public class SlidesTest extends LinearOpMode {
    Bot bot;
    private GamepadEx gp2;

    @Override
    public void runOpMode(){
        bot = Bot.getInstance(this);
        gp2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.slides.resetEncoder();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp2.readButtons();
            runSlides(gp2.getLeftY());
            telemetry.update();

            //dpad check
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.runToNextStageDown();
                //goes up
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slides.runToNextStageUp();
                //goes down
            }

            /*
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slides.runToLow();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                //change to System.getCurrentTimeMillis if this is buggy
                bot.slides.runToMid();
            }
             */

            bot.slides.periodic();
        }
    }

    private void runSlides(double power) {
        //manual slides test
        telemetry.addData("Gamepad Power", power);
        telemetry.addData("Slide Power Given",bot.slides.getManualPower());
        telemetry.addData("Slides Power", bot.slides.slidesMotor.getVelocity());

        bot.slides.runToManual(power);
    }
}
