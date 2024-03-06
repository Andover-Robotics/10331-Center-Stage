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
        bot.noodles.storage();

        waitForStart();

        bot.noodles.goToIntakePos();
        bot.reverseMotors();
        bot.slides.resetEncoder();
        bot.slides.resetProfiler();
        bot.slides.runTo(0);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("TeleOp has started");
            //if slides dont work run to top first
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


            drive();

            //intake
            if(gp2.wasJustPressed(GamepadKeys.Button.X)) {
                if(isIntake){
                    bot.stopIntake();
                    isIntake = false;
                }
                else {
                    bot.box.resetBox();
                    bot.intake();
                    isIntake = true;
                }
            }

            //reverse intake
            if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                bot.noodles.reverseIntake();
            }


            //fourbar and box outtake/storage position
            if(gp2.wasJustPressed(GamepadKeys.Button.A)) {

                if(isOuttakePosition) {
                    bot.box.resetBox();
                    bot.fourbar.storage();
                   // bot.noodles.intake();
                    isOuttakePosition=false;

                }

                else {
                    if(isIntake){
                        isIntake = false;
                        bot.stopIntake();
                    }
                    //outtake position
                    bot.fourbar.outtake();
                    isOuttakePosition=true;
                }
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
                bot.fourbar.outtake();
                if(isIntake){
                    isIntake = false;
                    bot.stopIntake();
                }
                isOuttakePosition = true;
                bot.box.depositFirstPixel();
                bot.box.depositSecondPixel();
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

            //shoot drone
            if (gp1.wasJustPressed(GamepadKeys.Button.B)){
                bot.drone.shoot();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)){
                bot.drone.reset();
            }

            telemetry.addData("noodle motor current:", bot.noodles.getIntakeMotorCurrent());
            telemetry.addData("FL motor current:",bot.getFLCurrent());
            telemetry.addData("FR motor current:",bot.getFRCurrent());
            telemetry.addData("BL motor current:",bot.getBLCurrent());
            telemetry.addData("BR motor current:",bot.getBRCurrent());
       //     telemetry.addData("right slides motor current:",bot.slides.getRightSlidesCurrent());
        //    telemetry.addData("mid slides motor current:",bot.slides.getMidSlidesCurrent());
            telemetry.addData("noodle motor power:", bot.noodles.getIntakeMotorPower());
            telemetry.addData("FL motor power:",bot.getFLPower());
            telemetry.addData("FR motor power:",bot.getFRPower());
            telemetry.addData("BL motor power :",bot.getBLPower());
            telemetry.addData("BR motor power:",bot.getBRPower());
       //     telemetry.addData("right slides motor power:",bot.slides.getRightSlidesPower());
         //   telemetry.addData("mid slides motor power:",bot.slides.getMidSlidesPower());
            telemetry.update();
            bot.slides.periodic();
        }
    }


    private void drive() {
        gp1.readButtons();
        bot.fixMotors();
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