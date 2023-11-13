package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainTeleop extends LinearOpMode {
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

        bot.reverseMotors();
        bot.slides.resetEncoder();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

        gp2.readButtons();
        gp1.readButtons();

        telemetry.addData("TeleOp has started","wheeeee");

        //divetrain movement
        drive();

        //intake
        if(gp2.wasJustPressed(GamepadKeys.Button.X)){
            if(isIntake){
                bot.stopIntake();
                isIntake=false;
            }
            else if(!isIntake){
                bot.intake();
                isIntake=true;
            }
        }

        //outtake position (without slide movement)
        if(gp2.wasJustPressed(GamepadKeys.Button.A)){
            bot.fourbar.outtake();
        }
        else if(gp2.wasJustPressed(GamepadKeys.Button.B)){
            bot.fourbar.storage();
        }

        telemetry.addData("box position", bot.fourbar.getBoxPos());
        telemetry.addData("fourbar position",bot.fourbar.getFourbarPos());
        telemetry.update();

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

    public void slidesRunToManual(double raw){
        bot.slides.runTo(raw*1800);
    }

    private void runSlides(){
        gp2.readButtons();
        double power = gp2.getLeftY();
        bot.slides.runToManual(power);
    }

}

