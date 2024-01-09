package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Fourbar {
    public final Servo fourbar;
    private final Servo angleBoxServo;

    //values ARE FINALLL DO NOT CHANGE
    private static double outtakeBox=0.51;

   // private static double boxAngleReadyForStorage=1.0;
    private static double storageBox=0.95;
    private static double outtake = 0.0;
    public static double storage = 0.7;
    //storage position is 0,1

    public static double fourbarPos=1.0;

    public static double boxInitialPos=0.5;
    public static boolean isOuttakePosition;


    public Fourbar(OpMode opMode) {
        fourbar = opMode.hardwareMap.servo.get("fourBarServo");
        angleBoxServo= opMode.hardwareMap.servo.get("boxAngleServo");
        fourbar.setDirection(Servo.Direction.FORWARD);
        isOuttakePosition= false;
    }

    public void outtake(){
        fourbar.setPosition(outtake);
        angleBoxServo.setPosition(outtakeBox);
        isOuttakePosition = true;
    }
    public void outtakeTest(){
        fourbar.setPosition(outtake);
        isOuttakePosition = true;
    }
    public void storageTest(){
        fourbar.setPosition(storage);
        isOuttakePosition = false;
    }
    public void outtakeBoxTest(){
        angleBoxServo.setPosition(outtakeBox);
        isOuttakePosition = true;
    }
    public void storageBoxTest(){
        angleBoxServo.setPosition(storageBox);
        isOuttakePosition = false;
    }

    public void storage(){
        //angleBoxServo.setPosition(boxAngleReadyForStorage);
       // angleBoxServo.setPosition(storageBox);
        fourbar.setPosition(storage);
        isOuttakePosition = false;
    }

    public void position(double fourbarPos, double box){
        angleBoxServo.setPosition(box);
        fourbar.setPosition(fourbarPos);
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
    public void incrementBoxAnglePosition(){
        boxInitialPos+=0.01;
        if(boxInitialPos>=1){
            boxInitialPos=1;
        }
        angleBoxServo.setPosition(boxInitialPos);
    }
    public void decrementBoxAnglePosition(){
        boxInitialPos-=0.01;
        if(boxInitialPos<=0){
            boxInitialPos=0;
        }
        angleBoxServo.setPosition(boxInitialPos);
    }
    public void incrementFourbarPosition(){
        fourbarPos+=0.1;
        if(fourbarPos>=1){
            fourbarPos=1;
        }
        fourbar.setPosition(fourbarPos);
    }
    public void decrementFourbarPosition(){
        fourbarPos-=0.1;
        if(fourbarPos<=0){
            fourbarPos=0;
        }
        fourbar.setPosition(fourbarPos);
    }
    public boolean getIsOuttakePos(){
        return isOuttakePosition;
    }

    public double getStoragePos(){
        return storage;
    }

    public double getOuttakePos(){
        return outtake;
    }
    public double getBoxStoragePos(){
        return storageBox;
    }

    public double getBoxOuttakePos(){
        return outtakeBox;
    }
    public double getBoxPos(){
        return angleBoxServo.getPosition();
    }
    public double getFourbarPos(){
        return fourbar.getPosition();
    }

}
