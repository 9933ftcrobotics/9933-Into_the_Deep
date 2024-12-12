package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.System_Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.System_Constants.FieldPositions;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem_CC;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Autonomous(name = "Auto to Teleop Test", group = "Autonomous", preselectTeleOp = "MainDrive_CC_after_Auto")

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

        TrajectoryActionBuilder scoreFirstTraj = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(FieldPositions.scoreSample.position, FieldPositions.scoreSample.heading);

        Action scoreFirst = scoreFirstTraj.build();

        Action turn_to_Zero = scoreFirstTraj.endTrajectory().fresh()
                .turnTo(Math.toRadians(-90))
                .build();



        int reqArmPos = 0, reqOutReq = 0;


        // actions that need to happen on init; for instance, a claw tightening.
        //claw.SetWristRight();


        arm.resetArm();
        arm.resetOutArm();
        arm.resetArmOffset();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(

                    new ParallelAction( //Prep Score
                            arm.setArmsAction(),
                            new SequentialAction(
                                   // new InstantAction(arm::startAuto),
                                    new ParallelAction(
                                        arm.setArmPosAction(DriveConstants.armSampleScoreHigh,DriveConstants.armOutSampleScoreHigh),
                                        new InstantAction(claw::SetWristCenter),
                                        scoreFirst
                                    ),
                                    new InstantAction(claw::grabberPlaceSlow),
                                    new SleepAction(1),
                                    new ParallelAction( //Prep Score
                                            new InstantAction(claw::grabberStop),
                                            arm.setArmPosAction(DriveConstants.armZero,DriveConstants.armOutZero),
                                            turn_to_Zero
                                    )
                                    //new InstantAction(arm::stopAuto)

                            )
                    )

        );
    }
}