package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;

@TeleOp(group = "test")
public class SlidesTest extends LinearOpMode {

    //unplugged port 0
    //unplugged port 1
    Bot bot;
    private GamepadEx gp2;

    @Override
    public void runOpMode(){
        bot = Bot.getInstance(this);
        gp2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.slides.resetEncoder();
        //bot.slides.runToStorage();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp2.readButtons();
            runSlides(gp2.getLeftY());

       /*     if(gp2.wasJustPressed(GamepadKeys.Button.A)) {
                bot.slides.test(0.25);
            }

            if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                bot.slides.test(0);
            }

            if(gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.slides.test(-0.25);
            }
        */

         //   bot.slides.periodic();


            //dpad check
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
             //   bot.slides.periodic();
                bot.slides.runToTop();
             //   bot.slides.periodic();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
             //   bot.slides.periodic();
                bot.slides.runToStorage();
             //   bot.slides.periodic();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
             //   bot.slides.periodic();
                bot.slides.runToLow();
             //   bot.slides.periodic();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
              //  bot.slides.periodic();
                bot.slides.runToMid();
              //  bot.slides.periodic();;
            }
            bot.slides.periodic();
        }
    }

    private void runSlides(double power) {
        bot.slides.runToManual(power);
       // bot.slides.periodic();
    }
}