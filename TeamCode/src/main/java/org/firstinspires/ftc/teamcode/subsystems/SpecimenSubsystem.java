package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import android.drm.DrmStore;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class SpecimenSubsystem extends SubsystemBase {
    //public static double upCurrent;
    private Motor leftClimb;

    private ServoEx leftPick;
    private ServoEx rightPick;

    private boolean outArmInPos, upArmInPos = false;

    double outPower = 0;

    private PIDController controller;

    private double targetPos = 0;
    public static int leftClimbCurret;

    public static int target = 0;
    public static int outTarget = 0;

    //private final double ticks_in_degree =  5281.1 / 360; //gobilda 30rpm encoder
    private final double ticks_in_degree =  8192 / 360; //rev thru bore

    public static double p = 0.0025, i = 0, d = 0.0004;
    public static double f = 0.07;

    public SpecimenSubsystem(Motor leftClimb, ServoEx leftPick, ServoEx rightPick) {
        leftClimb.setInverted(false);
        this.leftClimb = leftClimb;
        this.leftPick = leftPick;
        this.rightPick = rightPick;
        leftClimb.resetEncoder();
        controller = new PIDController(p,i,d);
    }

    public void setLeftClimbArm(int Pos) {
        leftClimbCurret = leftClimb.getCurrentPosition();
        targetPos = Pos;
        // set the run mode

        PIDController pid = new PIDController(p,i,d);
        double ff = Math.cos(Math.toRadians(Pos / ticks_in_degree))*f;

        // set the tolerance
        double tolerence = 4;   // allowed maximum error
        // perform the control loop
        //if (Math.abs(Pos-arm.getCurrentPosition()) > tolerence) {
        leftClimb.set(pid.calculate(leftClimb.getCurrentPosition(),Pos) + ff);
        upArmInPos = false;
        /*} else {
            arm.stopMotor(); // stop the motor
            upArmInPos = true;
        }*/

        /*controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();

        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree))*f;


        double power = pid + ff;

        arm.set(power);*/

    }

    public void leftServoClose() {
        leftPick.setPosition(0);
    }
    public void leftServoOpen() {
        leftPick.setPosition(0.4);
    }

    public void rightServoClose() {
        rightPick.setPosition(0);
    }
    public void rightServoOpen() {
        rightPick.setPosition(0.4);
    }





    /*public Action Sleep(int Sec) {
        wait(1);

        return upOutAuto(Pos);
    }*/

    public String[] getArmTelemetry() {
        return new String[]{
                ("Left Climb Arm Pos: " + String.valueOf(leftClimb.getCurrentPosition())),
                ("Target Left Climb Arm Pos: " + String.valueOf(targetPos))
        };
    }


    public class SpecimenPick implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setLeftClimbArm(0);
            Run = Run + 1;
            return !upArmInPos && Run < 30;
        }
    }
    public Action specimenPick() {
        return new SpecimenPick();
    }

    public class SpecimenPlace implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setLeftClimbArm(900);
            Run = Run + 1;
            return !upArmInPos && Run < 30;
        }
    }
    public Action specimenPlace() {
        return new SpecimenPlace();
    }

    public class SpecimenClip implements Action {
        int Run = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setLeftClimbArm(800);
            Run = Run + 1;
            return !upArmInPos && Run < 30;
        }
    }
    public Action specimenClip() {
        return new SpecimenClip();
    }



}