package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem_CC;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Autonomous(name = "Auto to Teleop Test", group = "Autonomous", preselectTeleOp = "MainDrive_CC")

public class Auto_to_Teleop_Test extends LinearOpMode {


    @Override
    public void runOpMode() {
        ArmSubsystem_CC arm = new ArmSubsystem_CC(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "armEnc", Motor.GoBILDA.RPM_312)
        );
        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );

        Pose2d initialPose = new Pose2d(32, 62, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        int reqArmPos = 0, reqOutReq = 0;


        // actions that need to happen on init; for instance, a claw tightening.
        //claw.SetWristRight();


        arm.resetArm();
        arm.resetOutArm();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        arm.autoArms();
//reqArmPos, reqOutReq
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .strafeToSplineHeading(new Vector2d(45, 45), Math.toRadians(50))
                        //.stopAndAdd(new arm.autoArms(DriveConstants.armOutSampleScoreHigh, DriveConstants.armOutSampleScoreHigh))
                        .build()



        );
    }
}