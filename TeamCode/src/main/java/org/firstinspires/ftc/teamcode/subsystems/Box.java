package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//link to intake, comment out breakbeam sensor

public class Box {
    private final CRServo wheelServo;
    private final Servo flapServo;

    private int numPixelsDeposited=0;
    private boolean boxFull;
    private final double flapClosed = 0;
    private final double flapOpen = 0.5;
   private ElapsedTime time;

    public Box(OpMode opMode) {
        wheelServo = opMode.hardwareMap.crservo.get("wheel servo");
        wheelServo.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo = opMode.hardwareMap.servo.get("flap servo");
        time = new ElapsedTime();
    }

    public void depositFirstPixel() {
        flapServo.setPosition(flapOpen);
        numPixelsDeposited = 1;
    }

    public void depositSecondPixel() {
        flapServo.setPosition(flapOpen);
        runWheel(false);
        numPixelsDeposited = 0;
    }
    public void secure() {
        time.reset();
        while(time.seconds() < 0.5) {
            runWheel(true);
        }
        wheelServo.setPower(0);
    }


    public void runWheel(boolean isHolding) {
        wheelServo.setDirection(DcMotorSimple.Direction.REVERSE);
        if(!isHolding){
            wheelServo.setPower(1);
        }
        else {
            wheelServo.setPower(-1);
        }
    }

    //wheel spins in reverse to keep the pixel from falling out
    // 2 servos - one for wheel, one for flap

    public void resetBox() {
        numPixelsDeposited = 0;
        flapServo.setPosition(flapClosed);
        wheelServo.setPower(0);
    }

    public boolean getIsFull(){
        return boxFull;
    }

    public void setIsFull(boolean isFull){
        boxFull = isFull;
    }


    public int getNumPixelsDeposited(){
        return numPixelsDeposited;
    }

}
