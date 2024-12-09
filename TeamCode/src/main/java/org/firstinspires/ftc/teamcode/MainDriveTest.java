package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp
@Disabled
public class MainDriveTest extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static final boolean FIELD_CENTRIC = true;

    boolean sampleScoring; //true = sample false = specimin

    boolean YIsPressed = false;

    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;

    boolean started = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER

        DriveSubsystem drive = new DriveSubsystem(
                new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312),
                new RevIMU(hardwareMap),
                hardwareMap.get(HuskyLens.class, "huskyLens")
        );

        ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_30),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312)
        );

        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );
        CameraSubsystem camera = new CameraSubsystem(

        );

        drive.setReadType(); //Set Husky Cam to color mode
        arm.resetOutArm();

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)


        // the extended gamepad object
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);


        waitForStart();
        //camera.initAprilTag();

        while (!isStopRequested()) {
            arm.armCurrent();
            telemetry.addData("Current", DriveConstants.armCurrent);
            CommandScheduler.getInstance().run();
            updateTelemetry(drive.getDriveTelemetry());


                claw.SetWristCenter();

                if (driver1.getButton(GamepadKeys.Button.DPAD_DOWN) || driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    arm.setArm(DriveConstants.armSamplePick);
                    arm.setOutArm(DriveConstants.armOutSamplePick);
                    claw.grabberPick();
                } else if (driver1.getButton(GamepadKeys.Button.DPAD_UP) || driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                    if (ArmSubsystem.upCurrent < DriveConstants.armSampleScoreHigh - 50) {
                        arm.setArm(DriveConstants.armSampleScoreHigh);
                        arm.setOutArm(0);
                    } else {
                        arm.setArm(DriveConstants.armSampleScoreHigh);
                        arm.setOutArm(DriveConstants.armOutSampleScoreHigh);
                    }
                } else if (driver1.getButton(GamepadKeys.Button.DPAD_RIGHT) || driver2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                    arm.setArm(DriveConstants.armSampleScoreLow);
                    arm.setOutArm(DriveConstants.armOutSampleScoreLow);
                } else {
                    arm.setArm(DriveConstants.armSampleRest);
                    arm.setOutArm(DriveConstants.armOutSampleRest);
                }

                if (driver1.getButton(GamepadKeys.Button.A) || driver2.getButton(GamepadKeys.Button.A)) {
                    claw.grabberPlace();
                } else {
                    if (!driver1.getButton(GamepadKeys.Button.DPAD_DOWN) && !driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                        claw.grabberStop();
                    }
                }

                    drive.drive(driver1.getLeftX(), driver1.getLeftY(), driver1.getRightX(), true);

        }
        drive.getCurrentPose();

        updateTelemetry(drive.getDriveTelemetry());

        telemetry.addData("out Current", ArmSubsystem.outCurrent);
        telemetry.addData("out Target", ArmSubsystem.outTarget);

        telemetry.update();


        //}
    }
    public void updateTelemetry(String[] telem) {
        for (String s : telem) {
            telemetry.addLine(s);
        }
    }

}