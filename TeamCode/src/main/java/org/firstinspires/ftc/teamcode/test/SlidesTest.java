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
        bot.slides.runToStorage();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp2.readButtons();
            runSlides(gp2.getLeftY());
            telemetry.update();

            //dpad check
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slides.periodic();
                bot.slides.runToTop();
                //dont question why the commands seem upside down... idk why
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.periodic();
                bot.slides.runToStorage();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slides.periodic();
                bot.slides.runToLow();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slides.periodic();
                bot.slides.runToMid();
            }

            bot.slides.periodic();
        }
    }

    private void runSlides(double power) {
        //manual slides test
        telemetry.addData("Gamepad Power", power);
        telemetry.addData("Slide Power Given",bot.slides.getManualPower());
        telemetry.addData("Slides Power", bot.slides.slidesMotor.getVelocity());
        bot.slides.periodic();
        bot.slides.runToManual(power);

    }
}