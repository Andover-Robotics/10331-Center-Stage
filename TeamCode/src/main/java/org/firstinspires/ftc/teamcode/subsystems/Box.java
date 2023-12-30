package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//link to intake, comment out breakbeam sensor

public class Box {
    private final CRServo wheelServo;
    private final CRServo flapServo;

    private int numPixelsDeposited=0;
    //  private final DigitalChannel breakbeamSensor;
    private boolean boxFull;
    //private int timesBroken;
    private final double flapClosed = 0;
    private final double flapOpen = 0.5;
    private ElapsedTime time;

    //boolean boxFull has to receive input from break beam sensor

    public Box(OpMode opMode) {
        wheelServo = opMode.hardwareMap.crservo.get("wheel servo");
        wheelServo.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo = opMode.hardwareMap.crservo.get("flap servo");
       /* breakbeamSensor = hardwareMap.get(DigitalChannel.class, "breakbeamSensor");
        breakbeamSensor.setMode(DigitalChannel.Mode.INPUT);
        */
        time = new ElapsedTime();
    }

    public void depositFirstPixel() {
        flapServo.setPower(flapOpen);
        // runWheel(true);
        numPixelsDeposited = 1;
    }

    public void depositSecondPixel() {
        flapServo.setPower(flapOpen);
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
        // timesBroken= 0;
        numPixelsDeposited = 0;
        flapServo.setPower(flapClosed);
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

