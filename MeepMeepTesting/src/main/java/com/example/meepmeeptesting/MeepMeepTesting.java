package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPoseBlueFar = new Pose2d(-36, 52, Math.toRadians(-90));
        Pose2d startPoseBlueClose = new Pose2d(10, 56, Math.toRadians(-90));

        Pose2d startPoseRedClose = new Pose2d(10, -52, Math.toRadians(90));

        Pose2d startPoseRedFar = new Pose2d(-34, -48, Math.toRadians(90));

        Vector2d purplePixelRightPosBlueClose = new Vector2d(56, 56);
        Vector2d purplePixelLeftPosBlueClose = new Vector2d(56, 56);
        Vector2d purplePixelCenterPosBlueClose = new Vector2d(56, 56);

        Vector2d purplePixelRightPosBlueFar = new Vector2d(56, 56);
        Vector2d purplePixelLeftPosBlueFar = new Vector2d(56, 56);
        Vector2d purplePixelCenterPosBlueFar = new Vector2d(56, 56);

        Vector2d purplePixelRightPosRedFar = new Vector2d(56, 56);
        Vector2d purplePixelLeftPosRedFar = new Vector2d(56, 56);
        Vector2d purplePixelCenterPosRedFar = new Vector2d(56, 56);

        Vector2d purplePixelRightPosRedClose = new Vector2d(56, 56);
        Vector2d purplePixelLeftPosRedClose = new Vector2d(56, 56);
        Vector2d purplePixelCenterPosRedClose = new Vector2d(56, 56);

        Vector2d scoreBackdropBlue = new Vector2d(42, 34);
        Vector2d scoreBackdropRed = new Vector2d(42, -34);


        Vector2d parkingPosBlue = new Vector2d(56, 60);

        Vector2d parkingPosRed = new Vector2d(54, -52);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRedClose)
                                .lineToConstantHeading(parkingPosRed)
                                .build()
                );





        //blue close is done
        //red close is done

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();



    /*     TrajectorySequence blueAllianceCloseApproachSpike = drive.trajectorySequenceBuilder(startPoseBlueClose)
                .splineTo(new Vector2d(10, 38), Math.toRadians(-90))
                .build();


        TrajectorySequence blueAllianceFarApproachSpike = drive.trajectorySequenceBuilder(startPoseBlueFar)
                .splineTo(new Vector2d(-36, 34), Math.toRadians(-90))
                .build();

        TrajectorySequence blueAllianceScore = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(scoreBackdropBlue)
                .build();

        TrajectorySequence blueAlliancePark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(parkingPosBlue)
                .build();

     */
    }
}