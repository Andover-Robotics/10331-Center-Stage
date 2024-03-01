package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


//link to intake, comment out breakbeam sensor

public class Box {
    public final CRServo wheelServo;
    public final CRServo flapServo;

    private int numPixelsDeposited=0;
  //  private final DigitalChannel breakbeamSensor;
    private boolean boxFull;
    //private int timesBroken;
   private ElapsedTime time;

    //boolean boxFull has to receive input from break beam sensor

    public Box(OpMode opMode) {

        //switched configs

        wheelServo = opMode.hardwareMap.crservo.get("wheel servo");
        wheelServo.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo = opMode.hardwareMap.crservo.get("flap servo");

        time = new ElapsedTime();
    }

    public void depositFirstPixel() {
        time.reset();
        while(time.seconds() < 1.5) {
            flapServo.setPower(1);
        }
        flapServo.setPower(0);
        numPixelsDeposited = 1;
    }

    public void depositSecondPixel() {
        time.reset();
        while(time.seconds() < 2) {
            flapServo.setPower(1);
            wheelServo.setPower(1);
        }
        flapServo.setPower(0);
        wheelServo.setPower(0);

        numPixelsDeposited = 0;
    }
    public void secure() {
        flapServo.setPower(0);
        time.reset();
        while(time.seconds() < 0.5) {
            wheelServo.setPower(-1);
        }
        wheelServo.setPower(0);
    }


    public void runWheel(boolean isHolding) {
        wheelServo.setDirection(DcMotorSimple.Direction.FORWARD);
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
       // timesBroken= 0;
        numPixelsDeposited = 0;
        flapServo.setPower(0);
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
