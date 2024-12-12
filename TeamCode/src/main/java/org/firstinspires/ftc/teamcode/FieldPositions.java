package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class FieldPositions {
    public static Pose2d prepToPickSecondSample = new Pose2d(27.75, 35.5,Math.toRadians(-26.5));
    public static Pose2d finishPickSecondSample = new Pose2d(32.75, 32.5,Math.toRadians(-26.5));
    public static Pose2d scoreSample = new Pose2d(51, 47,Math.toRadians(50));
    public static Pose2d prepToPickThirdSample = new Pose2d(37.6, 23.25,Math.toRadians(0));
    public static Pose2d finishPickThirdSample = new Pose2d(43, 23.25,Math.toRadians(0));
    public static Pose2d prepToPickFourthSample = new Pose2d(46.7, 23.25,Math.toRadians(0));
    public static Pose2d finishPickFourthSample = new Pose2d(51.8, 23.25,Math.toRadians(0));

    Pose2d prepPushFirstSpecimen = new Pose2d(-33.5, 20,Math.toRadians(-90));
    Pose2d startPushFirstSpecimen = new Pose2d(-45, 10,Math.toRadians(-90));
    Pose2d finishPushFirstSpecimen = new Pose2d(-42, 59,Math.toRadians(-90));
    Pose2d prepPushSecondSpecimen = new Pose2d(-45, 14,Math.toRadians(-90));
    Pose2d startPushSecondSpecimen = new Pose2d(-55.5, 10,Math.toRadians(-90));
    Pose2d finishPushSecondSpecimen = new Pose2d(-53, 57,Math.toRadians(-90));
    Pose2d prepPushThirdSpecimen = new Pose2d(-55, 12,Math.toRadians(-90));
    Pose2d startPushThirdSpecimen = new Pose2d(-60, 14,Math.toRadians(-90));
    Pose2d finishPushThirdSpecimen = new Pose2d(-60, 58,Math.toRadians(-90));
//public static int test = 0;

}
