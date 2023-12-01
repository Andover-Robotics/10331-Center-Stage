package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Bot;

@Autonomous
public class ResetEverything extends LinearOpMode {
    Bot bot;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = bot.getInstance(this);
        bot.resetEverything();
    }
}
