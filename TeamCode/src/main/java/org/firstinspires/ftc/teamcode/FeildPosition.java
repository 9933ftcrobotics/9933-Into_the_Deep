package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class FeildPosition {
    public static Pose2d prepToPickSecondSample = new Pose2d(26, 35,Math.toRadians(-31));
    public static Pose2d finishPickSecondSample = new Pose2d(31, 32,Math.toRadians(-31));
    public static Pose2d scoreSample = new Pose2d(49, 46,Math.toRadians(40));
    Pose2d prepToPickThirdSample = new Pose2d(36, 21.5,Math.toRadians(0));
    Pose2d finishPickThirdSample = new Pose2d(41.5, 21.5,Math.toRadians(0));
    Pose2d prepToPickFourthSample = new Pose2d(45, 21.5,Math.toRadians(0));
    Pose2d finishPickFourthSample = new Pose2d(51, 21.5,Math.toRadians(0));

    Pose2d prepPushFirstSpecimen = new Pose2d(-33.5, 20,Math.toRadians(-90));
    Pose2d startPushFirstSpecimen = new Pose2d(-45, 10,Math.toRadians(-90));
    Pose2d finishPushFirstSpecimen = new Pose2d(-42, 59,Math.toRadians(-90));
    Pose2d prepPushSecondSpecimen = new Pose2d(-45, 14,Math.toRadians(-90));
    Pose2d startPushSecondSpecimen = new Pose2d(-55.5, 10,Math.toRadians(-90));
    Pose2d finishPushSecondSpecimen = new Pose2d(-53, 57,Math.toRadians(-90));
    Pose2d prepPushThirdSpecimen = new Pose2d(-55, 12,Math.toRadians(-90));
    Pose2d startPushThirdSpecimen = new Pose2d(-60, 14,Math.toRadians(-90));
    Pose2d finishPushThirdSpecimen = new Pose2d(-60, 58,Math.toRadians(-90));



    Pose2d scoreFirstSpecimen = new Pose2d(2, 34,Math.toRadians(180));
    Pose2d pickPrep = new Pose2d(-49, 49,Math.toRadians(0));
    Pose2d pick = new Pose2d(-50, 60,Math.toRadians(0));
    Pose2d park = new Pose2d(-50, 60,Math.toRadians(-90));
//public static int test = 0;

}
