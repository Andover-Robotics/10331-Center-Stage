package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public class Noodles {
    //we need a sensor to sense how many pixels we are intaking!
    public final DcMotorEx noodleMotor;
    public final CRServo counterRoller;
    public final Servo extendOne;
    public final Servo extendTwo;
    private final OpMode opMode;
    private boolean isIntake;

    //values need to be runed
    private final double extendStoragePosition=0;
    private final double extendOuttakePosition=0.8;


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
        extendOne.setPosition(extendStoragePosition);
        extendTwo.setPosition(extendStoragePosition);
        isIntake= false;
        noodleMotor.setPower(0);
        counterRoller.setPower(0);
    }

    public void goToIntakePos(){
        extendOne.setPosition(extendOuttakePosition);
        extendTwo.setPosition(extendOuttakePosition);
    }


    public void reverseIntake(){
        noodleMotor.setDirection(DcMotorEx.Direction.FORWARD);
        noodleMotor.setPower(0.5);
        isIntake=false;
        noodleMotor.setPower(0.5);
        counterRoller.setDirection(DcMotorEx.Direction.REVERSE);
        counterRoller.setPower(1);
    }


    public boolean getIntakeState(){
        return isIntake;
    }




}
