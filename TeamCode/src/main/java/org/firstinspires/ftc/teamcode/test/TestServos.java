package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;



@TeleOp
public class TestServos extends LinearOpMode {

    Bot bot;
    private double driveSpeed = 1;
    private GamepadEx gp1;
    private GamepadEx gp2;
    public boolean isIntake=false;
    public boolean isOuttakePosition=false;

    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        boolean isRun=true;

        telemetry.addData("boxAnglePosition:", bot.fourbar.getBoxStoragePos());

        waitForStart();
        bot.reverseMotors();


        while (opModeIsActive() && !isStopRequested()) {

            gp2.readButtons();

            if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                if(isRun){
                    bot.box.wheelServo.setPower(1);
                    //power of 1 is intaking
                    isRun=false;
                }
                else{
                    bot.box.wheelServo.setPower(0);
                    isRun=true;
                }
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.X)){
               if(isRun){
                   bot.box.flapServo.setPower(1);
                   //power of 1 is depositing
                   isRun=false;
               }
               else{
                   bot.box.flapServo.setPower(0);
                   isRun=true;
               }
            }


        }

    }
}
