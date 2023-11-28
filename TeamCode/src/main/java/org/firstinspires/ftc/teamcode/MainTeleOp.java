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
    public boolean isOuttakePosition=false;


    @Override
    public void runOpMode() throws InterruptedException {

        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        telemetry.addData("boxAnglePosition:", bot.fourbar.getBoxStoragePos());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            gp2.readButtons();
            telemetry.addLine("TeleOp has started");

            //drivetrain movement works
            drive();

            //intake works
            if(gp2.wasJustPressed(GamepadKeys.Button.X)) {
                if(isIntake){
                    bot.stopIntake();
                    isIntake = false;
                    telemetry.addLine("Stopped Intaking");
                }
                else {
                    bot.intake();
                    isIntake = true;
                    telemetry.addLine("Currently intaking");
                }
                telemetry.update();
            }
            //consider doing while x is pressed

            //reverse intake
            if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                bot.noodles.reverseIntake();
            }

            //fourbar and box outtake/storage position
            if(gp2.wasJustPressed(GamepadKeys.Button.A)) {
                if(isOuttakePosition) {
                    //storage position
                    bot.box.resetBox();
                    bot.fourbar.storage();
                    isOuttakePosition=false;
                    telemetry.addLine("Currently in storage position");
                }
                else {
                    //outtake position
                    bot.fourbar.outtake();
                    isOuttakePosition=true;
                    telemetry.addLine("Currently in outtake position");
                    telemetry.update();
                }
                telemetry.update();
            }

            //button just deposits pixel, resets after second pixel
            if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                if(bot.box.getNumPixelsDeposited()==1){
                    bot.box.depositSecondPixel();
                    sleep(1000);
                    bot.box.resetBox();
                }
                else {
                    bot.box.depositFirstPixel();
                }
            }

            //fourbar and box (automatic deposit): deposits both pixels at same time
            if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                bot.fourbar.outtake();
                if(bot.fourbar.getIsOuttakePos()) {
                    isOuttakePosition = true;
                    bot.box.depositFirstPixel();
                    bot.box.depositSecondPixel();
                }
                telemetry.addLine("Currently in outtake position and deposited two pixels");
            }

            //manual movement of slides
            runSlides(gp2.getLeftY());

            //slide movement to preset values
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.runToNextStageUp();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slides.runToNextStageDown();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.B)){
                bot.drone.shoot();
                telemetry.addLine("Drone shooting");
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)){
                bot.drone.reset();
                telemetry.addLine("Drone resetting");
            }

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


    private void runSlides(double power) {
        bot.slides.runToManual(power);
    }

}