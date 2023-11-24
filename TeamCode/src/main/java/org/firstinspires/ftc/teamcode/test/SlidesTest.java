package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.subsystems.Slides.low;
import static org.firstinspires.ftc.teamcode.subsystems.Slides.storage;
import static org.firstinspires.ftc.teamcode.subsystems.Slides.top;

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
    private double doubleClickThreshold, lastMidPress;
    private boolean midPressed;




    @Override
    public void runOpMode(){
        bot = Bot.getInstance(this);
        gp2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.slides.resetEncoder();
        doubleClickThreshold = 0.2;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp2.readButtons();
            runSlides(gp2.getLeftY());
            telemetry.update();

            //dpad check
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.runToStorage();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slides.runToTop();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slides.runToLow();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                //change to System.getCurrentTimeMillis if this is buggy
                double currentTime = 0L;
                if(!midPressed) {
                    midPressed = true;
                    currentTime = bot.opMode.time;
                }

                if(currentTime - lastMidPress < doubleClickThreshold)
                    bot.slides.runToMid(2);
                else
                    bot.slides.runToMid(1);

                lastMidPress = bot.opMode.time;
            }

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
