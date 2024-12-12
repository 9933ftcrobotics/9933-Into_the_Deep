package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.System_Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.System_Constants.SpecimenConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem_CC;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenSubsystem;

@TeleOp
public class MainDrive_CC extends LinearOpMode {
    boolean wristCenter = true;


    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

    private static ElapsedTime rumble = new ElapsedTime();
    boolean warning = false;
    boolean endGame = false;

    double leftY, leftX, rightX;

    int ReqArmPos, ReqOutPos, ReqLeftArmPos, ReqRightArmPos;
    public static ArmSubsystem_CC arm;
    boolean Arm_Override_Active;
    boolean Clipping;
    boolean Climbing;
    boolean leftBumperPressed, modeSlow, modeSlow_for_Specimen;
    double slow = 0.5; //Percentage of how slow the "slow" mode is.
    double speed;
    @Override
    public void runOpMode() throws InterruptedException {

        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DriveSubsystem drive = new DriveSubsystem(
                new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312),
                new RevIMU(hardwareMap),
                hardwareMap.get(HuskyLens.class, "huskyLens")
        );

        arm = new ArmSubsystem_CC(
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

        //CameraSubsystem camera = new CameraSubsystem( );

        //drive.setReadType();
        if(!RobotInfo.PrevAuto) {
            arm.resetArm();
            arm.resetOutArm();
            arm.resetArmOffset();
            specimenSubsystem.resetArms();
        }

        //Tell the robot that the last thing we did was not Auto so most likely next time we're
        // starting with arms from store position
        RobotInfo.PrevAuto = false;


        rumble.reset();


        // the extended gamepad object
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        ReqArmPos = DriveConstants.armSampleRest;
        ReqOutPos = DriveConstants.armOutSampleRest;

        ReqLeftArmPos = SpecimenConstants.SpecimenRest;

        waitForStart();
        //camera.initAprilTag();

        while (!isStopRequested()) {
            rightX = driver1.getRightX();
            leftX = driver1.getLeftX();
            leftY = driver1.getLeftY();


            if (!modeSlow && !modeSlow_for_Specimen) {
                speed = 1;
            } else {
                speed = slow;
            }

            drive.drive(leftX * speed, leftY * speed, rightX * speed, true);


            //drive.getCurrentPose();

            updateTelemetry(drive.getDriveTelemetry());
            updateTelemetry(arm.getArmTelemetry());
            updateTelemetry(specimenSubsystem.getArmTelemetry());

            //When this CommandScheduler was enabled it slowly made
            // the arm PID worse, like we were building up garbage every time we ran
            //CommandScheduler.getInstance().run();


            //Speed Reduce
            if (driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && leftBumperPressed == false) {
                if (modeSlow == true) {
                    modeSlow = false;
                } else if (modeSlow == false) {
                    modeSlow = true;
                }
                leftBumperPressed = true;
            }

            if (!driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && !driver2.getButton(GamepadKeys.Button.LEFT_BUMPER) ) {
                leftBumperPressed = false;
            }

            if (driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.075) {
                claw.grabberPlaceToPower(driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            } else if (driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.075) {
                claw.grabberPlaceToPower(driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            } else if (driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.075) {
                claw.grabberPlaceToPower(-driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            } else if (driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.075) {
                claw.grabberPlaceToPower(-driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            } else if (!driver2.getButton(GamepadKeys.Button.A) && !driver2.getButton(GamepadKeys.Button.DPAD_DOWN)){
                claw.grabberStop();
            }


            specimenSubsystem.setArms();
            specimenSubsystem.setBothArmsPosition(ReqLeftArmPos,ReqRightArmPos);

            if(driver2.getButton(GamepadKeys.Button.B)){
                ReqLeftArmPos = SpecimenConstants.SpecimenPick;
                modeSlow_for_Specimen = true;
                Clipping = false;
                Climbing = false;
            }
            else if(driver2.getButton(GamepadKeys.Button.A)){
                ReqLeftArmPos = 0;
                Clipping = false;
                Climbing = false;
            }
            else if(driver2.getButton(GamepadKeys.Button.Y)){
                ReqLeftArmPos = SpecimenConstants.SpecimenDeliver;
                Clipping = true;
                Climbing = false;
            }
            else if(driver2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                Clipping = false;
                Climbing = true;
                ReqLeftArmPos = SpecimenConstants.Raise_arm_to_Climb_Left;
                ReqRightArmPos = SpecimenConstants.Raise_arm_to_Climb_Right;
            }
            else{
                ReqRightArmPos = SpecimenConstants.Lower_arm_to_Climb;
                if(!Clipping && !Climbing) {
                    ReqLeftArmPos = SpecimenConstants.SpecimenRest;
                } else if(Climbing){
                    ReqLeftArmPos = SpecimenConstants.Lower_arm_to_Climb;
                }
                else{
                    ReqLeftArmPos = SpecimenConstants.SpecimenClip;
                }
                modeSlow_for_Specimen = false;
            }

            if(driver2.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                specimenSubsystem.specimenClawClose();
            }
            else{
                specimenSubsystem.specimenClawOpen();
            }


            if (driver1.getButton(GamepadKeys.Button.X)) {
                claw.SetWristLeft();
            } else if (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                claw.SetWristCenter();
            }
            arm.setArmPositions(ReqArmPos,ReqOutPos);
            if(!Arm_Override_Active) {
                //by putting this out of the state machine we don't accidentally forget to call this
                // and send the arm into orbit because its still set to a constant power value instead
                // of still checking against the PID controllers.

                arm.setArms();
                if(driver1.getButton(GamepadKeys.Button.DPAD_UP) || driver1.getButton(GamepadKeys.Button.DPAD_DOWN) || driver1.getButton(GamepadKeys.Button.A) || driver1.getButton(GamepadKeys.Button.Y)){
                    Arm_Override_Active = true;
                }
                else if (driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                    ReqArmPos = DriveConstants.armSampleScoreHigh;
                    ReqOutPos = DriveConstants.armOutSampleScoreHigh;
                    modeSlow = true;
                }
                else if (driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    ReqArmPos = DriveConstants.armSamplePickFar;
                    ReqOutPos = DriveConstants.armOutSamplePickFar;
                    claw.grabberPick();
                    modeSlow = false;
                }
                else if (driver2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                    ReqArmPos = DriveConstants.armSamplePick;
                    ReqOutPos = DriveConstants.armOutSamplePick;
                    claw.grabberPick();
                    modeSlow = false;
                }

                else {
                    ReqArmPos = DriveConstants.armSampleRest;
                    ReqOutPos = DriveConstants.armOutSampleRest;

                    modeSlow = false;
                }
            }
            else{
                //have to call seperate arms because when overriding you want to be able to lift
                //and lower arm regardless where the out arm is

                arm.setArm();
                arm.setOutArm();

                if(driver1.getButton(GamepadKeys.Button.BACK)){
                    Arm_Override_Active = false;
                }
                else if(driver1.getButton(GamepadKeys.Button.START)){
                    Arm_Override_Active = false;
                    arm.resetOutArm();
                    arm.resetArm();
                }
                else if(driver1.getButton(GamepadKeys.Button.DPAD_UP)){
                    ReqArmPos = ReqArmPos + 10;
                }
                else if(driver1.getButton(GamepadKeys.Button.DPAD_DOWN)){
                    ReqArmPos = ReqArmPos - 10;
                }
                else if(driver1.getButton(GamepadKeys.Button.Y)){
                    ReqOutPos = ReqOutPos + 20;
                }
                else if(driver1.getButton(GamepadKeys.Button.A)){
                    ReqOutPos = ReqOutPos - 20;
                }
            }
            if (rumble.seconds() > 80 && !warning) {
                driver1.gamepad.rumbleBlips(3);
                warning = true;
            } else if (rumble.seconds() > 90 && !endGame) {
                driver1.gamepad.rumbleBlips(6);
                endGame = true;
            }
            telemetry.addData("Timmer For End Game", rumble);


            telemetry.update();


        }
    }
    public void updateTelemetry(String[] telem) {
        for (String s : telem) {
            telemetry.addLine(s);
        }
    }
}