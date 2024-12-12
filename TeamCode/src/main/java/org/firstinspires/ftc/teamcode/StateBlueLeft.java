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
@Autonomous(name = "StateBlueLeft", group = "Autonomous", preselectTeleOp = "MainDrive_CC_after_Auto")

public class StateBlueLeft extends LinearOpMode {


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

        TrajectoryActionBuilder ScoreFirst = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(FieldPositions.scoreSample.position, FieldPositions.scoreSample.heading);
        Action scoreFirst = ScoreFirst.build();

        TrajectoryActionBuilder PrepSecondPick = ScoreFirst.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.prepToPickSecondSample.position, FieldPositions.prepToPickSecondSample.heading);
        Action prepSecondPick = PrepSecondPick.build();

        TrajectoryActionBuilder SecondPick = PrepSecondPick.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.finishPickSecondSample.position, FieldPositions.finishPickSecondSample.heading);
        Action secondPick = SecondPick.build();

        TrajectoryActionBuilder SecondScore = SecondPick.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.scoreSample.position, FieldPositions.scoreSample.heading);
        Action secondScore = SecondScore.build();

        TrajectoryActionBuilder ThirdPrepPick = SecondScore.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.prepToPickThirdSample.position, FieldPositions.prepToPickThirdSample.heading);
        Action thirdPrepPick = ThirdPrepPick.build();

        TrajectoryActionBuilder ThirdPick = ThirdPrepPick.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.finishPickThirdSample.position, FieldPositions.finishPickThirdSample.heading);
        Action thirdPick = ThirdPick.build();

        TrajectoryActionBuilder ThirdScore = ThirdPick.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.scoreSample.position, FieldPositions.scoreSample.heading);
        Action thirdScore = ThirdScore.build();

        TrajectoryActionBuilder FourthPrepPick = ThirdScore.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.prepToPickFourthSample.position, FieldPositions.prepToPickFourthSample.heading);
        Action fourthPrepPick = FourthPrepPick.build();

        TrajectoryActionBuilder FourthPick = FourthPrepPick.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.finishPickFourthSample.position, FieldPositions.finishPickFourthSample.heading);
        Action fourthPick = FourthPick.build();

        TrajectoryActionBuilder FourthScore = FourthPick.endTrajectory().fresh()
                .strafeToSplineHeading(FieldPositions.scoreSample.position, FieldPositions.scoreSample.heading);
        Action fourthScore = FourthScore.build();




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
                                new SleepAction(0.5),
                                new ParallelAction( //Prep Score
                                        new InstantAction(claw::grabberPick),
                                        arm.setArmPosAction(DriveConstants.armSampleRest,DriveConstants.armOutSampleRest),
                                        prepSecondPick
                                ),
                                new ParallelAction( //Prep Score
                                        new InstantAction(claw::grabberPick),
                                        arm.setArmPosAction(DriveConstants.armSamplePick,DriveConstants.armOutSamplePick),
                                        secondPick
                                ),
                                new SleepAction(0.4),
                                new ParallelAction( //Prep Score
                                        new InstantAction(claw::grabberStop),
                                        arm.setArmPosAction(DriveConstants.armSampleScoreHigh,DriveConstants.armOutSampleScoreHigh),
                                        secondScore
                                ),
                                new InstantAction(claw::grabberPlaceSlow),
                                new SleepAction(0.5),
                                new ParallelAction( //Prep Score
                                        new InstantAction(claw::grabberPick),
                                        arm.setArmPosAction(DriveConstants.armSampleRest,DriveConstants.armOutSampleRest),
                                        thirdPrepPick
                                ),
                                new ParallelAction( //Prep Score
                                        arm.setArmPosAction(DriveConstants.armSamplePick,DriveConstants.armOutSamplePick),
                                        thirdPick
                                ),
                                new SleepAction(0.4),
                                new ParallelAction( //Prep Score
                                        new InstantAction(claw::grabberStop),
                                        arm.setArmPosAction(DriveConstants.armSampleScoreHigh,DriveConstants.armOutSampleScoreHigh),
                                        thirdScore
                                ),
                                new InstantAction(claw::grabberPlaceSlow),
                                new SleepAction(0.5),
                                new ParallelAction( //Prep Score
                                        new InstantAction(claw::grabberPick),
                                        arm.setArmPosAction(DriveConstants.armSampleRest,DriveConstants.armOutSampleRest),
                                        fourthPrepPick
                                ),
                                new ParallelAction( //Prep Score
                                        arm.setArmPosAction(DriveConstants.armSamplePick,DriveConstants.armOutSamplePick),
                                        fourthPick
                                ),
                                new SleepAction(0.4),
                                new ParallelAction( //Prep Score
                                        new InstantAction(claw::grabberStop),
                                        arm.setArmPosAction(DriveConstants.armSampleScoreHigh,DriveConstants.armOutSampleScoreHigh),
                                        fourthScore
                                ),
                                new InstantAction(claw::grabberPlaceSlow),
                                new SleepAction(0.5),
                                new ParallelAction( //Prep Score
                                        arm.setArmPosAction(DriveConstants.armZero,DriveConstants.armOutZero)
                                )
                                //new InstantAction(arm::stopAuto)

                        )
                )

        );
    }
}