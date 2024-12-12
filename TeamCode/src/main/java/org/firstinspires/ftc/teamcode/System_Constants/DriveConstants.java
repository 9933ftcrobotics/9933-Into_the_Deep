package org.firstinspires.ftc.teamcode.System_Constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class DriveConstants {

    public static double TICKS_PER_REV = 383.6;
    public static double WHEEL_DIAMETER = 0.1;
    public static double DISTANCE_PER_PULSE = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
    //TODO: Check units
    public static double TRACK_WIDTH = 6.75;

    public static double CENTER_WHEEL_OFFSET = 6;

    public static double MAX_VELOCITY = 1.5;
    public static double MAX_ACCELERATION = 1.5;

    public static double B = 2.0;
    public static double ZETA = 0.7;


    /*
    //Rev Thru Bore
    public static int armSamplePick = 50;
    public static int armSamplePickFar = 250;
    public static int armSampleRest = 300;
    public static int armSampleScoreLow = 1300;
    public static int armSampleScoreHigh = 2000;

    public static int armZero = 100;

    public static int armSpecimenPick = armSamplePick;
    public static int armSpecimenClip = 1450;
    public static int armSpecimenRest = 1100;*/

    //gobilda 30RPM Motor Encoder
    public static int armSamplePick = 235;
    public static int armSamplePickFar = 190;
    public static int armSampleRest = 500;
    public static int armSampleScoreLow = 1100;
    public static int armSampleScoreHigh = 1510;

    public static int armZero = 100;

    public static int armSpecimenPick = armSamplePick;
    public static int armSpecimenClip = 1350;
    public static int armSpecimenRest = 1100;

    public static int armOutSamplePick = 929;
    public static int armOutSamplePickFar = 1400;
    public static int armOutSampleRest = 0;
    public static int armOutSampleScoreLow = 1200;
    public static int armOutSampleScoreHigh = 2100;

    public static int armOutZero = 0;

    public static int armOutSpecimenPick = armOutSamplePick;
    public static int armOutSpecimenClip = 900;
    public static int armOutSpecimenRest = 410;


    //public static int armStartHang = 875;
    //public static int armFinishHang = 125;

    public static int armCurrent;

    public static boolean leftBumperPressed = false;
    public static boolean rightBumperPressed = false;

    public static boolean huskyReading = false;


    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    public static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public static AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public static boolean targetFound = false;    // Set to true when an AprilTag target is detected
    public static boolean foundTag = false; //This will help with picking a tag and using it only
    public static double xCameraPos; ///This is the position of the robot along the wall NOTE: This is NOT x in RoadRunner. It is along the wall of the apriltag.
    public static double yCameraPos; ///This is the position of the robot away from the wall
    public static double yawCameraPos; //This is the rotation of the robot in perspective of the wall. Facing wall is 0.
    public static boolean driving = false; //If using roadrunner path

    public static int runTrajectory;

    public static boolean foundBlock;
    public static int colorID = 2;
    public static int autoPickPosition;
    public static double xDis;
    public static double yDis;
    public static boolean roadRunning = false;

    public static double angle;



}
