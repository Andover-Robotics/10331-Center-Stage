package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Bot;

@TeleOp
public class ResetEverything extends LinearOpMode {
    Bot bot;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            bot = bot.getInstance(this);
            bot.resetEverything();
        }
    }
}
