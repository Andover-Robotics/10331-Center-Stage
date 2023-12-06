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

    private int numPixelsDeposited;
  //  private final DigitalChannel breakbeamSensor;
    private boolean boxFull;
    //private int timesBroken;
    private final double flapClosed = 0;
    private final double flapOpen = 1;
   // private ElapsedTime time;

    //boolean boxFull has to receive input from break beam sensor

    public Box(OpMode opMode) {
        wheelServo = opMode.hardwareMap.crservo.get("wheel servo");
        wheelServo.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo = opMode.hardwareMap.servo.get("flap servo");
       /* breakbeamSensor = hardwareMap.get(DigitalChannel.class, "breakbeamSensor");
        breakbeamSensor.setMode(DigitalChannel.Mode.INPUT);
        */
       // time = new ElapsedTime();
    }

    public void depositFirstPixel() {
        flapServo.setPosition(flapOpen);
        wheelServo.setPower(0);
        numPixelsDeposited = 1;
    }

    public void depositSecondPixel() {
        flapServo.setPosition(flapOpen);
        runWheel(false);
        numPixelsDeposited = 2;
    }
    public void secure() throws InterruptedException {
        //time.reset();
        wheelServo.setPower(-0.2);
        wheelServo.setPower(0);
    }


    public void runWheel(boolean isHolding) {
        if(!isHolding) wheelServo.setPower(1);
        else wheelServo.setPower(-1);
    }

    //wheel spins in reverse to keep the pixel from falling out
    // 2 servos - one for wheel, one for flap

    public void resetBox() {
       // timesBroken= 0;
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

    /*
    public void checkBeam(){
        boolean isBeamBroken = breakbeamSensor.getState();
        if (isBeamBroken) {
            telemetry.addData("Status", "Object detected!");
            timesBroken++;
        } else {
            telemetry.addData("Status", "No object detected");
        }
        if(timesBroken ==2){
           setIsFull(true);
        }
    }

     */

    public int getNumPixelsDeposited(){
        return numPixelsDeposited;
    }

}
