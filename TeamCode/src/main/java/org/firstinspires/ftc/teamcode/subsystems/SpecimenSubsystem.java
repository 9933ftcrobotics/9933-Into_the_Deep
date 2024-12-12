package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.RobotInfo;
import org.firstinspires.ftc.teamcode.System_Constants.SpecimenConstants;

public class SpecimenSubsystem extends SubsystemBase {
    //public static double upCurrent;
    private Motor leftClimb;
    private Motor rightClimb;
    private ServoEx leftPick;
    private ServoEx rightPick;

    private boolean outArmInPos, upArmInPos = false;

    double outPower = 0;

    private PIDController leftController, rightController;

    private double leftClimbtargetPos, rightClimbtargetPos;
    public static int leftClimbCurrent,rightClimbCurrent;


    public static double p = 0.006, i = 0, d = 0;


    public SpecimenSubsystem(Motor leftClimb, Motor rightClimb, ServoEx leftPick, ServoEx rightPick) {
        leftClimb.setInverted(true);
        leftPick.setInverted(true);
        this.leftClimb = leftClimb;
        this.rightClimb = rightClimb;
        this.leftPick = leftPick;
        this.rightPick = rightPick;
        //leftClimb.resetEncoder();
        leftController = new PIDController(p,i,d);
        rightController = new PIDController(p,i,d);
    }

    public void setArms() {
        leftClimbCurrent = leftClimb.getCurrentPosition();
        rightClimbCurrent = rightClimb.getCurrentPosition();

        // set the run mode

        PIDController left_pid = new PIDController(p,i,d);
        PIDController right_pid = new PIDController(p,i,d);

        // perform the control loop
        leftClimb.set(left_pid.calculate(leftClimbCurrent,leftClimbtargetPos));
        rightClimb.set(right_pid.calculate(rightClimbCurrent,rightClimbtargetPos));


    }

    public void setLeftArmPosition(double leftClimbArmReqPos){
        leftClimbtargetPos = leftClimbArmReqPos;
    }
    public void setRightArmPosition(double rightClimbArmReqPos){
        rightClimbtargetPos = rightClimbArmReqPos;
    }
    public void setBothArmsPosition(double leftClimbArmReqPos, double rightClimbArmReqPos){
        setLeftArmPosition(leftClimbArmReqPos);
        setRightArmPosition(rightClimbArmReqPos);
    }
    public void specimenClawClose() {

        leftPick.setPosition(SpecimenConstants.LeftClawGrab);
        rightPick.setPosition(SpecimenConstants.RightClawGrab);
    }

    public void specimenClawOpen() {

        leftPick.setPosition(SpecimenConstants.LeftClawRelease);
        rightPick.setPosition(SpecimenConstants.RightClawRelease);
    }

    public Action setLeftArmPosAction(int ReqArmTarget_In) {

        return new Action() {
            int PrevArmPos, PrevOutArmPos, timeout_count;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                leftClimbtargetPos = ReqArmTarget_In;

                //setArms();
                boolean ArmInPos = false;

                if ((leftClimb.getCurrentPosition() < leftClimbtargetPos + 50) && (leftClimb.getCurrentPosition() > leftClimbtargetPos - 50)) {
                    ArmInPos = true;
                }

                return !ArmInPos;
            }
        };
    }

    public Action setArmsAction() {


        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setArms();

                packet.put("Left Climb Arm Pos", leftClimb.getCurrentPosition());
                packet.put("Left Climb Target Pos", leftClimbtargetPos);
                packet.put("Right Climb Arm Pos", rightClimb.getCurrentPosition());
                packet.put("Right Climb Target Pos", rightClimbtargetPos);

                return true;
            }
        };
    }

    public void leftServoClose() {
        leftPick.setPosition(SpecimenConstants.LeftClawGrab);
    }
    public void leftServoOpen() {
        leftPick.setPosition(SpecimenConstants.LeftClawRelease);
    }

    public void rightServoClose() {rightPick.setPosition(SpecimenConstants.RightClawGrab);}
    public void rightServoOpen() {
        rightPick.setPosition(SpecimenConstants.RightClawRelease);
    }

    public void resetArms() {
        leftClimb.resetEncoder();
        rightClimb.resetEncoder();
    }

    public String[] getArmTelemetry() {
        return new String[]{
                ("Left Climb Arm Pos: " + String.valueOf(leftClimb.getCurrentPosition())),
                ("Target Left Climb Arm Pos: " + String.valueOf(leftClimbtargetPos))
        };
    }


}