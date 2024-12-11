package org.firstinspires.ftc.teamcode.subsystems;

import static com.acmerobotics.roadrunner.Math.clamp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.RobotInfo;

public class ArmSubsystem_CC extends SubsystemBase {

    private Motor arm;
    private Motor outArm;
    private Motor armEnc;

    double outPower = 0;

    private PIDController armPID,outPID;

    public static int outCurrent;
    public static int armCurrent;


    public static int ArmTarget = 0;
    public static int OutTarget = 0;

    private final double ticks_in_degree =  5281.1 / 360; //gobilda 30rpm encoder
    private final double ticks_in_degree_rev =  8192 / 360; //rev thru bore

    private final double motor_to_rev_ratio = 5281.1/8192;

    public static double arm_p = 0.0125, arm_i = 0.06, arm_d = 0.00075;
    public static double arm_f = 0.08;

    public static double out_p = 0.006, out_i = 0, out_d = 0;

    public static int ReqArmTarget = 0, ReqOutTarget = 0;

    public static boolean AutoRunning;

    double armInitDelay,PrevArmPos;



    public ArmSubsystem_CC(Motor arm, Motor outArm, Motor armEnc) {
        arm.setInverted(false);
        armEnc.setInverted(true);
        outArm.setInverted(true);
        this.arm = arm;
        this.outArm = outArm;
        this.armEnc = armEnc;

        armPID = new PIDController(arm_p,arm_i,arm_d);
        outPID = new PIDController(out_p,out_i,out_d);

    }

    public void setArms() {
        armCurrent = arm.getCurrentPosition();
        outCurrent = outArm.getCurrentPosition();
        if (!RobotInfo.arminitialized){
            if ((PrevArmPos > armCurrent-5) && (PrevArmPos < armCurrent+5)) {
                if(armInitDelay > 10){
                    RobotInfo.armOffset = armCurrent - (int)(armEnc.getCurrentPosition()*motor_to_rev_ratio);
                    RobotInfo.arminitialized = true;
                }
                else {armInitDelay++;}
            }
            else{
                PrevArmPos = armCurrent;
                armInitDelay = 0;
            }
        }


        if(outCurrent < 300) {
            ArmTarget = ReqArmTarget+RobotInfo.armOffset;
        }

        if(Math.abs(armCurrent-ArmTarget)<300){
            OutTarget = ReqOutTarget;
        }

        armPID.setPID(arm_p,arm_i,arm_d);

        double armpid = armPID.calculate(armCurrent,ArmTarget);
        double armff = Math.cos(Math.toRadians(ArmTarget / ticks_in_degree))*arm_f;

        double power = armpid + armff;

        power = clamp(power,-0.4,1);
        arm.set(power);

        outPID.setPID(out_p,out_i,out_d);

        double outpid = outPID.calculate(outCurrent,OutTarget);
        outpid = clamp(outpid,-0.65,0.65);
        outArm.set(outpid);


    }

  public Action setArmPosAction(int ReqArmTarget_In, int ReqOutTarget_In) {

      return new Action() {
          int PrevArmPos, PrevOutArmPos,timeout_count;
          @Override
          public boolean run(@NonNull TelemetryPacket packet) {

              ReqArmTarget = ReqArmTarget_In;
              ReqOutTarget =ReqOutTarget_In;
              //setArms();
              boolean ArmInPos = false;

              if((arm.getCurrentPosition() < ReqArmTarget+20) && (arm.getCurrentPosition() > ReqArmTarget-20)){


                  if((outArm.getCurrentPosition() < ReqOutTarget+50) && (outArm.getCurrentPosition() > ReqOutTarget-50)){
                      ArmInPos = true;
                  }
              }
              return !ArmInPos;
          }
      };
  }


    public Action setArmsAction() {

        AutoRunning = true;
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setArms();

                packet.put("Arm Pos", arm.getCurrentPosition());
                packet.put("Arm Target Pos", ReqArmTarget);
                packet.put("Out Pos", outArm.getCurrentPosition());
                packet.put("Out Target Pos", ReqOutTarget);
                packet.put("Initialized", RobotInfo.arminitialized);
                packet.put("Offset", RobotInfo.armOffset);
                packet.put("AutoRunning", AutoRunning);
                if(!AutoRunning) {
                    arm.set(0);
                    outArm.set(0);
                }
                return AutoRunning;
            }
        };
    }
public void setArmPositions(int ReqArmTarget_In, int ReqOutTarget_In){

    ReqArmTarget = ReqArmTarget_In;
    ReqOutTarget =ReqOutTarget_In;
}
    public void setArm() {

        armPID.setPID(arm_p, arm_i, arm_d);
        armCurrent = arm.getCurrentPosition();
        ArmTarget = ReqArmTarget;
        double armpid = armPID.calculate(armCurrent, ArmTarget);
        double armff = Math.cos(Math.toRadians(ArmTarget / ticks_in_degree)) * arm_f;


        double power = armpid + armff;

        power = clamp(power,-0.4,1);
        arm.set(power);
    }

    public void setOutArm() {
        outPID.setPID(out_p,out_i,out_d);
        outCurrent = outArm.getCurrentPosition();
        OutTarget = ReqOutTarget;

        double outpid = outPID.calculate(outCurrent,OutTarget);
        outpid = clamp(outpid,-0.65,0.65);
        outArm.set(outpid);

    }

    public void powerArm(double Power) {
        arm.set(Power);
    }

    public void powerOutArm(double Power) {
        outArm.set(Power);
    }

    public void armCurrent() {
        armCurrent = arm.getCurrentPosition();
    }

    public void resetOutArm() {outArm.resetEncoder();}
    public void resetArm() {
        arm.resetEncoder();
        armEnc.resetEncoder();
    }
    public void resetArmOffset(){
        RobotInfo.armOffset = 0;
        RobotInfo.arminitialized = false;
    }
    public Action stopAuto(){
        AutoRunning = false;
        return stopAuto();
    }

    public Action startAuto(){
        AutoRunning = true;
        return startAuto();
    }

    public String[] getArmTelemetry() {
        return new String[]{
                ("Current Arm Motor Pos: " + String.valueOf(arm.getCurrentPosition())),
                ("Current Arm Pos: " + String.valueOf(armEnc.getCurrentPosition()*motor_to_rev_ratio)),
                ("Arm Power: " + String.valueOf(arm.get())),
                ("Target Arm Pos: " + String.valueOf(ArmTarget)),
                ("Current Out Arm Pos: " + String.valueOf(outArm.getCurrentPosition())),
                ("Target Out Arm Pos: " + String.valueOf(OutTarget))

        };
    }


}

/*
Floor Pickup:
arm encoder, 242
out motor, 929


*/
