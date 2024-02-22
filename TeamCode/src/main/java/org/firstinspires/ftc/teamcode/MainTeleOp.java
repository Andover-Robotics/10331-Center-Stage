package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class MainTeleOp extends LinearOpMode {
    Bot bot;
    private double driveSpeed = 1;
    private GamepadEx gp1;
    private GamepadEx gp2;
    public boolean isIntake=false;
    public boolean isOuttakePosition=false;
    public boolean isIncrementFourbar=true;


    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);


        telemetry.addData("boxAnglePosition:", bot.fourbar.getBoxStoragePos());


        waitForStart();
        bot.reverseMotors();
        bot.slides.resetEncoder();
        bot.slides.resetProfiler();
        bot.slides.runToStorage();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("TeleOp has started");
            //if slides dont work run to storage first
            gp2.readButtons();
            runSlides(gp2.getLeftY());
           /*
           CONTROLS:
           GP1:
               Start button: Resets everything
               Left joystick X axis: strafe
               Left joystick Y axis: forward backward
               Right joystick X axis: turn
               B: Drone Shooting
               A: Drone Resetting


           GP2:
               X button:
                   clicked once: runs intake
                   clicked twice: stops intake
               Right Bumper: Reverses intake
               A:
                   if in outtake pos, returns to storage position
                   if in intake pos, goes to outtake position
               Y: deposits the pixels one at a time
               B: deposits both pixels at the same time
               Start button: Feeder bot mode
               DPAD UP: Run slides to top
               DPAD DOWN: Run slides to storage
               DPAD LEFT: Run slides to low stage
               DPAD RIGHT: Run slides to mid stage
               Left Joystick Y axis: Manual slides operation
            */


            //drivetrain movement works
            drive();


            if(gp1.wasJustPressed(GamepadKeys.Button.START)) {
                //   bot.resetEverything();
                isIncrementFourbar=!isIncrementFourbar;
            }


            //intake works
            if(gp2.wasJustPressed(GamepadKeys.Button.X)) {
                if(isIntake){
                    bot.stopIntake();
                    isIntake = false;
                    telemetry.addData("Stopped Intaking", isIntake);
                }
                else {
                    bot.box.resetBox();
                    bot.intake();
                    isIntake = true;
                    telemetry.addData("Currently Intaking", isIntake);
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
                    //  bot.noodles.intake();
                    isOuttakePosition=false;
                    telemetry.addLine("Currently in storage position");

                }

                else {
                    if(isIntake){
                        isIntake = false;
                        bot.stopIntake();
                    }
                    //outtake position
                    bot.fourbar.outtake();
                    isOuttakePosition=true;
                    telemetry.addLine("Currently in outtake position");
                    telemetry.update();
                }

            /*    if(isIncrementFourbar){
                    bot.fourbar.incrementBoxAnglePosition();
                }
                else{
                    bot.fourbar.decrementBoxAnglePosition();
                }

             */
                telemetry.update();
            }


            //button just deposits pixel, resets after second pixel
            if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                if(bot.box.getNumPixelsDeposited()==1){
                    bot.box.depositSecondPixel();
                    bot.box.resetBox();
                }
                else {
                    bot.box.depositFirstPixel();
                }
            }


            //fourbar and box (automatic deposit): deposits both pixels at same time
            if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                time.reset();
                bot.fourbar.outtake();
                if(isIntake){
                    isIntake = false;
                    bot.stopIntake();
                }
                isOuttakePosition = true;
                bot.box.depositFirstPixel();
                bot.box.depositSecondPixel();
                telemetry.addLine("Currently in outtake position and deposited two pixels");
            }


            //feeder bot -> deposit pixel
            if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                bot.noodles.reverseIntake();
                time.reset();
                while(time.seconds() < 5) {
                    bot.box.runWheel(true);
                }
                bot.box.runWheel(false);
            }


//            //slide movement to preset values
            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {;
                bot.slides.runToTop();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.runToStorage();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slides.runToLow();
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slides.runToMid();}

            if (gp1.wasJustPressed(GamepadKeys.Button.B)){
                bot.drone.shoot();
                telemetry.addLine("Drone shooting");
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)){
                bot.drone.reset();
                telemetry.addLine("Drone resetting");
            }

            telemetry.addData("box position", bot.fourbar.getBoxPos());
            telemetry.addData("fourbar position",bot.fourbar.getFourbarPos());
            telemetry.addData("slides position", bot.slides.getCurrentPosition());
            telemetry.update();

            bot.slides.periodic();
        }
    }


    private void drive() {
        gp1.readButtons();
        driveSpeed = 1;
        driveSpeed *= 1 - 0.9 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);

        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed / 1.7
        );
    }

    private void runSlides(double power) {
        bot.slides.runToManual(power*-0.5);
    }
}