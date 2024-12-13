package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.System_Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.System_Constants.SpecimenConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem_CC;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenSubsystem;

@TeleOp
public class Dry_Cycle_CC extends LinearOpMode {

    private static ElapsedTime timer = new ElapsedTime();

    double prev_time, cycles, TimeElapsed;
    int state, ReqArmPos, ReqOutPos;
    double ReqLeftClimbPos, ReqRightClimbPos;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//cycles = 100;

        ArmSubsystem_CC arm = new ArmSubsystem_CC(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312)
        );

        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );

        SpecimenSubsystem specimenSubsystem = new SpecimenSubsystem(
                new Motor(hardwareMap, "leftClimb", Motor.GoBILDA.RPM_30),
                new Motor(hardwareMap, "rightClimb", Motor.GoBILDA.RPM_30),
                new SimpleServo(hardwareMap, "leftPick", 0,1),
                new SimpleServo(hardwareMap, "rightPick", 0,1));

        arm.resetOutArm();
        specimenSubsystem.resetArms();

        ReqArmPos = DriveConstants.armSampleRest;
        ReqOutPos = DriveConstants.armOutSampleRest;

        waitForStart();

        while (!isStopRequested()) {

            //CommandScheduler.getInstance().run();

            //by putting this out of the state machine we don't accidentally forget to call this
            // and send the arm into orbit because its still set to a constant power value instead
            // of still checking against the PID controllers.
            arm.setArmPositions(ReqArmPos,ReqOutPos);
            arm.setArms();

            specimenSubsystem.setBothArmsPosition(ReqLeftClimbPos, ReqRightClimbPos);
            specimenSubsystem.setArms();

            TimeElapsed = timer.seconds() - prev_time;

            switch(state){

                //Starting State to bring wrist to center in case it starts in tucked starting position
                case 0:
                    claw.SetWristCenter();
                    prev_time = timer.seconds();
                    state = 10;
                    break;

                //Arm to rest position, wait 3s, turn wrist right
                case 10:
                    ReqArmPos = DriveConstants.armSampleRest;
                    ReqOutPos = DriveConstants.armOutSampleRest;
                    if(TimeElapsed > 2){
                        claw.SetWristRight();
                        specimenSubsystem.specimenClawClose();
                        prev_time = timer.seconds();
                        state = 20;
                    }
                    break;

                //wait 2s, return wrist to center
                case 20:
                    if(TimeElapsed > 1){
                        ReqLeftClimbPos = SpecimenConstants.SpecimenDeliver;
                        claw.SetWristCenter();
                        prev_time = timer.seconds();
                        state = 30;
                    }
                    break;

                //wait 2s
                case 30:
                    if(TimeElapsed > 1){
                        claw.grabberPick();

                        ReqLeftClimbPos = SpecimenConstants.SpecimenClip;
                        prev_time = timer.seconds();
                        state = 33;
                    }
                    break;
                case 33:
                    if(TimeElapsed > 2){
                        ReqLeftClimbPos = SpecimenConstants.SpecimenRest;
                        specimenSubsystem.specimenClawOpen();
                        claw.grabberStop();
                        prev_time = timer.seconds();
                        state = 40;
                    }
                    break;
                //Raise arm, wait 5s, set wrist right
                case 40:

                    ReqArmPos = DriveConstants.armSampleScoreHigh;
                    ReqOutPos =DriveConstants.armOutSampleScoreHigh;

                    if(TimeElapsed > 3){
                        prev_time = timer.seconds();
                        claw.SetWristRight();
                        state = 50;
                    }
                    break;

                //wait 2s, wrist back to center
                case 50:

                    ReqLeftClimbPos = SpecimenConstants.SpecimenPick;
                    if(TimeElapsed > 1){
                        prev_time = timer.seconds();
                        claw.SetWristCenter();
                        state = 53;
                    }
                    break;

                //wait 2s, wrist back to center
                case 53:
                    specimenSubsystem.specimenClawClose();
                    if(TimeElapsed > 2){
                        prev_time = timer.seconds();
                        claw.grabberPlace();
                        state = 55;
                    }
                    break;

                case 55:

                    ReqLeftClimbPos = SpecimenConstants.Lower_arm_to_Climb;

                    if(TimeElapsed > 2){
                        prev_time = timer.seconds();
                        claw.grabberStop();
                        specimenSubsystem.specimenClawOpen();
                        state = 60;
                    }
                    break;

                //wait 2s, count cycle and restart back
                case 60:
                    if(TimeElapsed > 2){

                        ReqLeftClimbPos = SpecimenConstants.Raise_arm_to_Climb_Left;
                        ReqRightClimbPos = SpecimenConstants.Raise_arm_to_Climb_Right;
                        specimenSubsystem.specimenClawClimb();
                        prev_time = timer.seconds();
                        state = 70;
                    }
                    break;
                case 70:
                    if(TimeElapsed > 5){

                        ReqLeftClimbPos = SpecimenConstants.Lower_arm_to_Climb;
                        ReqRightClimbPos = SpecimenConstants.Lower_arm_to_Climb;

                        specimenSubsystem.specimenClawOpen();
                        prev_time = timer.seconds();
                        cycles = cycles + 1;
                        state = 10;
                    }
                    break;
            }


            telemetry.addData("Timer", timer);
            telemetry.addData("Prev Time", prev_time);
            telemetry.addData("Elapsed Time", TimeElapsed);

            telemetry.addData("Cycles: ", cycles);

            telemetry.addData("State: ", state);
            //updateTelemetry(drive.getDriveTelemetry());
            updateTelemetry(arm.getArmTelemetry());
            telemetry.update();


        }
    }
    public void updateTelemetry(String[] telem) {
        for (String s : telem) {
            telemetry.addLine(s);
        }
    }
}