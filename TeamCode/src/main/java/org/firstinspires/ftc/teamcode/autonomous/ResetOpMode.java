package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;

@TeleOp
public class ResetOpMode extends LinearOpMode {
    Bot bot;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = bot.getInstance(this);
        bot.resetEverything();
    }
}