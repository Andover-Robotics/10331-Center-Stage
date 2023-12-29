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
    private ElapsedTime time;


    @Override
    public void runOpMode(){
        bot = Bot.getInstance(this);
        gp2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.slides.resetEncoder();
        bot.slides.runToStorage();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            time = new ElapsedTime();
            gp2.readButtons();
           // runSlides(gp2.getLeftY());

            /*
            if(gp2.wasJustPressed(GamepadKeys.Button.A)) {
                bot.slides.test(0.5);
            }

            if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                bot.slides.test(0);
            }

             */


            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                //first stage
                while(time.seconds() <= 3) bot.slides.test(0.5);
                time.reset();
            } else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                while(time.seconds() <= 3) bot.slides.test(-0.5);
                time.reset();
            }


            //dpad check
           /* if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
             //   bot.slides.periodic();
                bot.slides.runToTop();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
             //   bot.slides.periodic();
                bot.slides.runToStorage();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
              //  bot.slides.periodic();
                bot.slides.runToLow();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
               // bot.slides.periodic();
                bot.slides.runToMid();
            }
            */
           // bot.slides.periodic();
        }
    }

    private void runSlides(double power) {
        //manual slides test
        telemetry.addData("Gamepad Power", power);
        telemetry.addData("Slide Power Given",bot.slides.getManualPower());
        //telemetry.addData("Slides Power", bot.slides.slidesMotor.getVelocity());
        bot.slides.runToManual(power);
        bot.slides.periodic();
    }
}