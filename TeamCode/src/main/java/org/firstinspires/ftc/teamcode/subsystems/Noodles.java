package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Noodles {
    //we need a sensor to sense how many pixels we are intaking!
    public final DcMotorEx noodleMotor;
    public final CRServo counterRoller;
    private final OpMode opMode;
    private boolean isIntake;

    public Noodles(OpMode opMode) {
        this.opMode = opMode;
        noodleMotor= opMode.hardwareMap.get(DcMotorEx.class, "noodles motor");
      counterRoller= opMode.hardwareMap.get(CRServo.class, "counter roller");
      counterRoller.setDirection(CRServo.Direction.FORWARD);
        noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        isIntake = false;
    }

    public void intake(){
      counterRoller.setDirection(CRServo.Direction.FORWARD);
        noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        noodleMotor.setPower(1);
     counterRoller.setPower(1);
        isIntake=true;
    }

    public void intake(double power){
      counterRoller.setDirection(CRServo.Direction.FORWARD);
        noodleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        noodleMotor.setPower(power);
     counterRoller.setPower(1);
        isIntake=true;
    }

    public void stop(){
        isIntake= false;
        noodleMotor.setPower(0);
      counterRoller.setPower(0);
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
