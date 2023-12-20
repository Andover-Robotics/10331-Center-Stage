package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagsDetection{


    public static double fx = 1078.03779;
    public static double fy = 1084.50988;
    public static double cx = 580.850545;
    public static double cy = 245.959325;

    // UNITS ARE METERS
    public static double tagSize = 0.032;

    static AprilTagsPipeline pipeline = new AprilTagsPipeline(tagSize, fx, fy, cx, cy);

    static final double FEET_PER_METER = 3.28084;
    static final double PIXELS_PER_METER = 3779.5275591;

    //will have to change this with the webcam


    // UNITS ARE METERS
    //might have to change this? not sure what size the tags will be


    static int ONE = 1;
    static int TWO = 2;
    static int THREE = 3;


    private static AprilTagDetection tagOfInterest = null;

    public static void detectTag(){

        //telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry();


            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if(currentDetections.size() != 0){
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections){
                    if(tag.id == ONE || tag.id == TWO || tag.id == THREE){
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound){
                    telemetry.addData("Tag of interest is in sight!\n\nLocation data:", "");
                    tagToTelemetry(tagOfInterest);
                }
                else{
                    telemetry.addData("Don't see tag of interest :(", "");

                    if(tagOfInterest == null){
                        telemetry.addData("(The tag has never been seen)", ".");
                    }
                    else{
                        telemetry.addData("\nBut we HAVE seen the tag before; last seen at", "" );
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else{
                telemetry.addData("Don't see tag of interest :(", "");

                if(tagOfInterest == null){
                    telemetry.addData("(The tag has never been seen)", "");
                }
                else{
                    telemetry.addData("\nBut we HAVE seen the tag before; last seen at:", "");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();


        //update telemetry
        if(tagOfInterest != null){
            telemetry.addData("Tag snapshot:\n", "");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else{
            telemetry.addData("No tag snapshot available, it was never sighted during the init loop :(", "");
            telemetry.update();
        }
    }
    public static AprilTagDetection getTagOfInterest(){
        detectTag();
        return tagOfInterest;
    }

     public double calcDistToTag(){
        detectTag();
        double distance = (tagSize * Math.sqrt(fx*fy))/(2*tagSize*PIXELS_PER_METER);
        return distance;
    }




    static void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addData("\nDetected tag ID=%d", detection.id);
        telemetry.addData("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER);
        telemetry.addData("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER);
        telemetry.addData("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER);
        //telemetry.addData(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        //telemetry.addData(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addData(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.update();

    }
    public static AprilTagDetection getTag(){
        return tagOfInterest;
    }






}

class AprilTagException extends Exception{
    public AprilTagException(){
    }

    public AprilTagException(String message)
    {
        super(message);
    }
}

