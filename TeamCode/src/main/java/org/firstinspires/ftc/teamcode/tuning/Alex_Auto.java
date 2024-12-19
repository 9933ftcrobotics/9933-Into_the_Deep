package org.firstinspires.ftc.teamcode.tuning;

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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.System_Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.System_Constants.FieldPositions;
import org.firstinspires.ftc.teamcode.System_Constants.SpecimenConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem_CC;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenSubsystem;

@Config
@Autonomous(name = "Alex Breaks The Robot", group = "Autonomous", preselectTeleOp = "MainDrive_CC_after_Auto")

public class Alex_Auto extends LinearOpMode {


    @Override
    public void runOpMode() {
        DcMotor leftClimb;
        leftClimb = hardwareMap.get(DcMotor.class, "leftClimb");
leftClimb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
leftClimb.setTargetPosition(0);
leftClimb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
leftClimb.setPower(1);
        ArmSubsystem_CC arm = new ArmSubsystem_CC(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312)
        );

        SpecimenSubsystem specimenSubsystem = new SpecimenSubsystem(
                new Motor(hardwareMap, "leftClimb", Motor.GoBILDA.RPM_30),
                new Motor(hardwareMap, "rightClimb", Motor.GoBILDA.RPM_30),
                new SimpleServo(hardwareMap, "leftPick", 0,1),
                new SimpleServo(hardwareMap, "rightPick", 0,1));

        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );

        Pose2d initialPose = new Pose2d(-2, 62, Math.toRadians(180));
        Pose2d approachChamber = new Pose2d(0, 50, Math.toRadians(180));
        Pose2d scoreChamber = new Pose2d(0, 29.25, Math.toRadians(180));


        Pose2d approachIntake = new Pose2d(-50, 50, Math.toRadians(-2));
        Pose2d intakeHuman = new Pose2d(-50, 62.5, Math.toRadians(3));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder scoreFirstTraj = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(approachChamber.position.x,approachChamber.position.y))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(scoreChamber.position.x,scoreChamber.position.y));

        Action scoreChamberFromStart = scoreFirstTraj.build();

        TrajectoryActionBuilder pickUpFromScoreTraj = scoreFirstTraj.endTrajectory().fresh()
                .strafeTo(new Vector2d(approachChamber.position.x,approachChamber.position.y))
                .waitSeconds(0.5)
                .splineToLinearHeading(approachIntake, approachIntake.heading)
                .waitSeconds(0.5)
                .turnTo(intakeHuman.heading)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(intakeHuman.position.x,intakeHuman.position.y));

        Action pickUpFromScore = pickUpFromScoreTraj
                .build();

        Action scoreFromPickUp = pickUpFromScoreTraj.endTrajectory().fresh()
                .strafeTo(new Vector2d(approachIntake.position.x,approachIntake.position.y))
                .waitSeconds(0.5)
                .splineToLinearHeading(approachChamber, approachChamber.heading)
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(177))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(scoreChamber.position.x,scoreChamber.position.y))
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

                    new SequentialAction( //Prep Score
                            new InstantAction(specimenSubsystem::specimenClawClose),
                            new ParallelAction(
                                    scoreChamberFromStart,
                                    new InstantAction(() -> leftClimb.setTargetPosition(SpecimenConstants.SpecimenDeliver))
                            ),
                            new SequentialAction(
                                    new InstantAction(() -> leftClimb.setTargetPosition(SpecimenConstants.SpecimenClip)),
                                    new SleepAction(1),
                                    new InstantAction(specimenSubsystem::specimenClawOpen)
                            ),
                            new SleepAction(1),
                            new ParallelAction(
                                    pickUpFromScore,
                                    new InstantAction(() -> leftClimb.setTargetPosition(SpecimenConstants.SpecimenPick))
                            ),
                            new SleepAction(1),
                            new InstantAction(specimenSubsystem::specimenClawClose),
                            new SleepAction(1),

                            new ParallelAction(
                                    scoreFromPickUp,
                                    new InstantAction(() -> leftClimb.setTargetPosition(SpecimenConstants.SpecimenDeliver))
                            ),
                            new SequentialAction(
                                    new InstantAction(() -> leftClimb.setTargetPosition(SpecimenConstants.SpecimenClip)),
                                    new SleepAction(1),
                                    new InstantAction(specimenSubsystem::specimenClawOpen)
                            ),
                            new SleepAction(1)
                    )

        );
    }
}