package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class TeamPropDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    Mat HSV = new Mat();
    MatOfPoint biggest;

    public static final int minimumWidth = 60;
    public static int width = 0;
    public enum TeamProp{
        ONLEFT,
        ONRIGHT,
        MIDDLE,
        NOTDETECTED
    }
    private enum Alliance{
        BLUE,
        RED,
    }

    private static TeamProp teamPropLocation= TeamProp.NOTDETECTED;
    private static Alliance alliance = Alliance.BLUE;


    //have to change values
    public static double lowH, lowS, lowV, highH, highS, highV;

    public static Scalar lowHSV;
    public static Scalar highHSV;


    public TeamPropDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {


        if(alliance== TeamPropDetectionPipeline.Alliance.BLUE){
    /*    lowH = 75;
        lowS = 110;
        lowV = 150;
        highH = 181;
        highS = 255;
        highV = 255;

     */
            lowH = 100;
            lowS = 0;
            lowV = 158;
            highH = 137;
            highS = 71;
            highV = 227;
        }
        else{
            lowH = 160;
            lowS = 83;
            lowV = 144;
            highH = 180;
            highS = 181;
            highV = 223;
        }
        lowHSV= new Scalar(lowH, lowS, lowV);
        highHSV= new Scalar(highH, highS, highV);

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);


        //have to change values (divide screen into three)
        Rect frontRect = new Rect(0, 1, 640, 719);
        //    Rect leftRect = new Rect(426, 1, 426, 719);
        Rect rightRect = new Rect(640,1,640,719);

        //red
        //    Imgproc.rectangle(input, leftRect, new Scalar(255, 0, 0), 5);

        //blue
        Imgproc.rectangle(input, rightRect, new Scalar(0, 0, 255), 5);

        //green
        Imgproc.rectangle(input, frontRect, new Scalar(0,255,0), 5);

        Core.inRange(HSV, lowHSV, highHSV, HSV);

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(HSV, contours, new Mat(), 0,1);

        if (!contours.isEmpty()) {

            //orders contours in array from big to small(by width)
            contours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width)));

            biggest = contours.get(0);

            // turns biggest contour into a rectangle
            Rect rect = Imgproc.boundingRect(biggest);
            width = rect.width;


            if (width > minimumWidth) {

                // puts green border around contours
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 6);

                double midpointRect = rect.tl().x + width/2.0;

                // checks if contour is within boundaries of any rectangle (left, right, front)

                if (midpointRect > rightRect.tl().x && midpointRect < rightRect.br().x) {
                    teamPropLocation = TeamProp.ONRIGHT;
                } else if (midpointRect < frontRect.tl().x && midpointRect > frontRect.br().x){
                    teamPropLocation = TeamProp.MIDDLE;
                }
                else{
                    teamPropLocation = TeamProp.ONLEFT;
                }

            }
        }

        HSV.release();
        return input;
    }
    public void setAlliance(int a){
        if(a==1){
            alliance = Alliance.RED;
        }
        else{
            alliance = Alliance.BLUE;
        }
    }
    public TeamProp getTeamPropLocation(){
        return teamPropLocation;
    }

}

class PropException extends Exception{
    public PropException(){
    }

    public PropException(String message)
    {
        super(message);
    }
}