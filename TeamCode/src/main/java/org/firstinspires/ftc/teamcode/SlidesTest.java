package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
            gp2.readButtons();
            runSlides();
            telemetry.update();

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

            bot.slides.periodic();
        }
    }

    private void runSlides() {
        double power = gp2.getLeftY();
        telemetry.addData(
                "Gamepad Power",power
        );
        telemetry.addData("Slide Power Given",bot.slides.manualPower);
//        if(power == 0){
//            bot.slides.brake();
//        }
        telemetry.addData("Slides Power", bot.slides.slidesMotor.getVelocity());
        bot.slides.runToManual(power);
    }
}
