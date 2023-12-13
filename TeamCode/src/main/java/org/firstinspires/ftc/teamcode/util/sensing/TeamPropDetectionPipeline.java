package org.firstinspires.ftc.teamcode.util.sensing;

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
    public static TeamProp teamPropLocation= TeamProp.NOTDETECTED;
    public static Alliance alliance = Alliance.BLUE;


    //have to change values
    public static double lowH, lowS, lowV, highH, highS, highV;

    public static Scalar lowHSV;
    public static Scalar highHSV;


    public TeamPropDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {

        if(alliance== Alliance.BLUE){
            lowH = 160;
            lowS = 80;
            lowV = 80;
            highH =200;
            highS = 120;
            highV = 120;
        }
        else{
            lowH = 280;
            lowS = 75;
            lowV = 80;
            highH =320;
            highS = 105;
            highV = 120;
        }
        lowHSV= new Scalar(lowH, lowS, lowV);
        highHSV= new Scalar(highH, highS, highV);

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        //have to change values (divide screen into three)
        Rect rightRect = new Rect(750, 1, 529, 719);
        Rect leftRect = new Rect(1, 1, 479, 719);
        Rect frontRect = new Rect(1,1,1,1);

        Imgproc.rectangle(input, leftRect, new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(input, rightRect, new Scalar(0, 0, 255), 5);
        Imgproc.rectangle(input, frontRect, new Scalar(255, 255, 0), 5);

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

                if (midpointRect > leftRect.tl().x && midpointRect < leftRect.br().x) {
                    teamPropLocation = TeamProp.ONLEFT;
                } else if (midpointRect > rightRect.tl().x && midpointRect < rightRect.br().x) {
                    teamPropLocation = TeamProp.ONRIGHT;
                } else if (midpointRect < frontRect.tl().x && midpointRect > frontRect.br().x){
                    teamPropLocation = TeamProp.MIDDLE;
                }

            }
            else {
                teamPropLocation = TeamProp.NOTDETECTED;
                telemetry.addLine("TeamProp not detected");
            }
        }

        else {
            teamPropLocation = TeamProp.NOTDETECTED;
            telemetry.addLine("TeamProp not detected");
        }

        HSV.release();
        return input;
    }
    public void setAlliance(int a){
        if(a==1){
            TeamPropDetectionPipeline.alliance = Alliance.RED;
        }
        else{
            TeamPropDetectionPipeline.alliance = Alliance.BLUE;
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