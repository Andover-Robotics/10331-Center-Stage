package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Bot;

@Config
@TeleOp(group="DroneTest")
public class DroneTest extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    @Override
    public void runOpMode() throws InterruptedException {
        bot=Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.B)){
                bot.drone.shoot();
                telemetry.addLine("Drone shooting");
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)){
                bot.drone.reset();
                telemetry.addLine("Drone resetting");
            }
            //probably not the actual buttons lol
        }
    }
}
