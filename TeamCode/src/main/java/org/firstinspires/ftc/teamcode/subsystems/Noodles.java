package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class Noodles {

    public final DcMotorEx noodleMotor;
    public final CRServo counterRoller;
    public final Servo extendOne;
    public final Servo extendTwo;
    private final OpMode opMode;
    private boolean isIntake;


    public double extendOnePos=0.625;
    public double extendTwoPos=0.9;
    //0.45 (storage left)
    //1.0 (storage right)


    //values need to be runed
    private final double extendOneStoragePosition=1.0;
    private final double extendTwoStoragePosition=0.45;
    private final double extendOneOuttakePosition=0.625;
    private final double extendTwoOuttakePosition=0.9;


    public Noodles(OpMode opMode) {
        this.opMode = opMode;
        noodleMotor= opMode.hardwareMap.get(DcMotorEx.class, "noodles motor");
        counterRoller= opMode.hardwareMap.get(CRServo.class, "counter roller");
        extendOne= opMode.hardwareMap.get(Servo.class, "extensionRight");
        extendTwo= opMode.hardwareMap.get(Servo.class, "extensionLeft");
        noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        isIntake = false;
    }


    public void intake(){
        counterRoller.setDirection(CRServo.Direction.FORWARD);
        noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        noodleMotor.setPower(-0.5);
        counterRoller.setPower(1);
        isIntake=true;
    }


    public void intake(double power){
        counterRoller.setDirection(CRServo.Direction.FORWARD);
        noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        noodleMotor.setPower(-power);
        counterRoller.setPower(1);
        isIntake=true;
    }


    public void stop(){
        isIntake= false;
        noodleMotor.setPower(0);
        counterRoller.setPower(0);
    }
    public void storage(){
        extendOne.setPosition(extendOneStoragePosition);
        extendTwo.setPosition(extendTwoStoragePosition);
        isIntake= false;
        noodleMotor.setPower(0);
        counterRoller.setPower(0);
    }

    public void goToIntakePos(){
        extendOne.setPosition(extendOneOuttakePosition);
        extendTwo.setPosition(extendTwoOuttakePosition);
    }


    public void reverseIntake(){
        noodleMotor.setDirection(DcMotorEx.Direction.FORWARD);
        noodleMotor.setPower(0.5);
        isIntake=false;
        noodleMotor.setPower(0.5);
        counterRoller.setDirection(DcMotorEx.Direction.REVERSE);
        counterRoller.setPower(1);
    }


    public void incrementServoRight(){
        extendOnePos+=0.05;
        if(extendOnePos>=1){
            extendOnePos=1;
        }
        extendOne.setPosition(extendOnePos);
    }
    public void decrementServoRight(){
        extendOnePos-=0.05;
        if(extendOnePos<=0){
            extendOnePos=0;
        }
        extendOne.setPosition(extendOnePos);
    }
    public void incrementServoLeft(){
        extendTwoPos+=0.05;
        if(extendTwoPos>=1){
            extendTwoPos=1;
        }
        extendTwo.setPosition(extendTwoPos);
    }
    public void decrementServoLeft(){
        extendTwoPos-=0.05;
        if(extendTwoPos<=0){
            extendTwoPos=0;
        }
        extendTwo.setPosition(extendTwoPos);
    }



    public boolean getIntakeState(){
        return isIntake;
    }




}
