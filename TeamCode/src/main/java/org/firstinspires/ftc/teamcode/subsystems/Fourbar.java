package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*public class Fourbar {
    private final Servo fourbar;
    private final Servo angleBoxServo;

    //values need to be changed
    private static double outtakeBox=0.5;
    private static double outtake = 1;

    //approximating angle as 180 degrees
    public static double storage = 0;
    public static boolean isOuttakePosition;


    public Fourbar(OpMode opMode) {
        fourbar = opMode.hardwareMap.servo.get("fourBarServo");
        angleBoxServo= opMode.hardwareMap.servo.get("boxAngle servo");
        fourbar.setDirection(Servo.Direction.FORWARD);
        isOuttakePosition= false;
    }

    public void outtake(){
        angleBoxServo.setPosition(outtakeBox);
        fourbar.setPosition(outtake);
        isOuttakePosition = true;
    }

    public void storage(){
        angleBoxServo.setPosition(0);
        fourbar.setDirection(Servo.Direction.REVERSE);
        fourbar.setPosition(storage);
        isOuttakePosition = false;
    }

    public void runManualOuttake(double position){
        angleBoxServo.setPosition(outtakeBox);
        fourbar.setPosition(position);
    }

    public void runManualStorage(double position){
        angleBoxServo.setPosition(0);
        fourbar.setDirection(Servo.Direction.REVERSE);
        fourbar.setPosition(position);
    }

    public boolean getIsOuttakePos(){
        return isOuttakePosition;
    }

}

 */
