package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.Temperature;
import org.firstinspires.ftc.teamcode.Bot;

@TeleOp(name="FourbarTest")
public class FourbarTest extends LinearOpMode {
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
                bot.fourbar.storage();
                telemetry.addLine("Currently in storage position");
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                bot.fourbar.outtake();
                telemetry.addLine("Currently in outtake position");
            }
        }
    }
}
