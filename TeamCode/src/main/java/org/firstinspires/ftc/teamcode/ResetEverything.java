package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
