package org.firstinspires.ftc.teamcode.System_Constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class SpecimenConstants {

    public static int SpecimenPick = 600;
    public static int SpecimenRest = 900;
    public static int SpecimenDeliver = 2400;
    public static int SpecimenClip = SpecimenDeliver - 500;
    public static int Raise_arm_to_Climb_Left = 4031;
    public static int Raise_arm_to_Climb_Right = 4290;
    public static int Lower_arm_to_Climb = 0;


    public static double LeftClawGrab = 0.4;
    public static double LeftClawRelease = 0;
    public static double LeftClawClimb = 0;


    public static double RightClawGrab = 0.3;
    public static double RightClawRelease = 0;
    public static double RightClawClimb = 0.6;








}
