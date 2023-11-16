package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Slides.low;
import static org.firstinspires.ftc.teamcode.subsystems.Slides.storage;
import static org.firstinspires.ftc.teamcode.subsystems.Slides.top;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class
SlidesTest extends LinearOpMode {
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
            runSlides();
            telemetry.update();


            /*
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                bot.slides.runToStorage();
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                bot.slides.runToTop();
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                bot.slides.runToLow();
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                bot.slides.runToMid();
            }
             */

            /*
            Stuff to test immediately after runToManual works
            1) bot.slides.runTo(top);
            2) bot.slides.runTo(storage);
            3) bot.slides.runTo(mid_1); bot.slides.runTo(mid_2)
            4) bot.slides.runTo(storage);
            5) bot.slides.runTo(low);
            6) bot.slides.runTo(storage);
             */

            bot.slides.periodic();
        }
    }

    private void runSlides() {
        //manual slides test
        double power = gp2.getLeftY();

        telemetry.addData("Gamepad Power", power);
        telemetry.addData("Slide Power Given",bot.slides.getManualPower());
        telemetry.addData("Slides Power", bot.slides.slidesMotor.getVelocity());

        bot.slides.runToManual(power);
    }
}
