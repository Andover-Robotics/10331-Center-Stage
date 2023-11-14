package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SlidesTest extends LinearOpMode {
    Bot bot;
    private GamepadEx gp2;


    @Override
    public void runOpMode(){
        bot = Bot.getInstance(this);
        gp2 = new GamepadEx(gamepad2);
        bot.slides.resetEncoder();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp2.readButtons();
            telemetry.addLine("OpMode has started");
            runSlides();

            /*

            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                bot.slides.runTo(1);
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                bot.slides.runTo(2);
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                bot.slides.runTo(3);
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                bot.slides.runTo(4);
            }

             */

        }
    }

    private void runSlides() {
        double power = gp2.getLeftY();
        bot.slides.runToManual(power);
    }
}
