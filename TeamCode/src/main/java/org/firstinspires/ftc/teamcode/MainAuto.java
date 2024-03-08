package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class MainAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            time.reset();

            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
                while(time.seconds() < 5) {
                    bot.strafeRight();
                }
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
                while(time.seconds() < 5) {
                    bot.strafeLeft();
                }
            }
        }


    }




}
