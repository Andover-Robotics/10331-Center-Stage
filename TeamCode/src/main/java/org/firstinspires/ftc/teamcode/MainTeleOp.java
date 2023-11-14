package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    Bot bot;
    private double driveSpeed = 1;
    private GamepadEx gp1;
    private GamepadEx gp2;
    public boolean isIntake=false;


    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        telemetry.addData("boxAnglePosition:", bot.fourbar.getBoxStoragePos());

        bot.resetEverything();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

        gp2.readButtons();
        gp1.readButtons();

        telemetry.addLine("TeleOp has started");

        //drivetrain movement
        drive();

        //intake
        if(gp2.wasJustPressed(GamepadKeys.Button.X)){
            if(isIntake){
                bot.stopIntake();
                isIntake = false;
                telemetry.addLine("Stopped Intaking");
            }
            else{
                bot.intake();
                isIntake = true;
                telemetry.addLine("Currently intaking");
            }
            telemetry.update();
        }

        //fourbar and box position
        if(gp2.wasJustPressed(GamepadKeys.Button.A)){
            bot.fourbar.outtake();
        }
        else if(gp2.wasJustPressed(GamepadKeys.Button.B)){
            bot.fourbar.storage();
        }

        //slide movement to preset values
        if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                bot.slides.runToStorage();
        }
        else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                bot.slides.runToTop();
        }
        else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                bot.slides.runToLow();
        }
        else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                bot.slides.runToMid();
        }

        //manual movement of slides
        runSlides();

        bot.slides.periodic();

        telemetry.addData("box position", bot.fourbar.getBoxPos());
        telemetry.addData("fourbar position",bot.fourbar.getFourbarPos());
        telemetry.update();
    }


}

    private void drive() {
        driveSpeed = 1;

        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);

        bot.driveRobotCentric(-driveVector.getX() * driveSpeed,
                -driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed / 1.7
        );
    }


    private void runSlides() {
        double power = gp2.getLeftY();
        telemetry.addData("Gamepad Power", power);

        telemetry.addData("Slide Power Given",bot.slides.manualPower);
//        if(power == 0){
//            bot.slides.brake();
//        }
        telemetry.addData("Slides Power", bot.slides.slidesMotor.getVelocity());
        bot.slides.runToManual(power);
    }

}

