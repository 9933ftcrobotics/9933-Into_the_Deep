package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FeildPosition;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous(name = "Test Blue Left", group = "Autonomous")

public class TestBlueLeft extends LinearOpMode {


    @Override
    public void runOpMode() {
        ArmSubsystem arm = new ArmSubsystem(
                new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312)
        );
        ClawSubsystem claw = new ClawSubsystem(
                new CRServo(hardwareMap, "grabber"),
                new SimpleServo(hardwareMap, "wrist", 0,1)
        );

        Pose2d initialPose = new Pose2d(32, 62, Math.toRadians(-90));
        Pose2d secondPos = new Pose2d(FeildPosition.scoreSample.position, FeildPosition.scoreSample.heading);
        Pose2d thirdPos = new Pose2d(FeildPosition.prepToPickSecondSample.position, FeildPosition.prepToPickSecondSample.heading);
        Pose2d fourthPos = new Pose2d(FeildPosition.finishPickSecondSample.position, FeildPosition.finishPickSecondSample.heading);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder scoreFirst = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(FeildPosition.scoreSample.position, FeildPosition.scoreSample.heading);

        TrajectoryActionBuilder grabSecondPrep = drive.actionBuilder(secondPos)
                .strafeToSplineHeading(FeildPosition.prepToPickSecondSample.position, FeildPosition.prepToPickSecondSample.heading);

        TrajectoryActionBuilder grabSecond = drive.actionBuilder(thirdPos)
                .strafeToSplineHeading(FeildPosition.finishPickSecondSample.position, FeildPosition.finishPickSecondSample.heading);

        TrajectoryActionBuilder scoreSecond = drive.actionBuilder(fourthPos)
                .strafeToSplineHeading(FeildPosition.scoreSample.position, FeildPosition.scoreSample.heading);


        TrajectoryActionBuilder WaitScore = drive.actionBuilder(initialPose)
                .waitSeconds(0.6);
        TrajectoryActionBuilder WaitScoreLong = drive.actionBuilder(initialPose)
                .waitSeconds(0.6);
        TrajectoryActionBuilder MidWait = drive.actionBuilder(initialPose)
                .waitSeconds(1);
        TrajectoryActionBuilder WaitPick = drive.actionBuilder(initialPose)
                .waitSeconds(0.7);
        TrajectoryActionBuilder WaitLong = drive.actionBuilder(initialPose)
                .waitSeconds(30);



        // actions that need to happen on init; for instance, a claw tightening.
        //claw.SetWristRight();

        Action firstScore = scoreFirst.build();
        Action prepGrabSecond = grabSecondPrep.build();
        Action secondGrab = grabSecond.build();
        Action secondScore = scoreSecond.build();

        Action waitPick = WaitPick.build();
        Action waitScore = WaitScore.build();
        Action waitScoreLong = WaitScoreLong.build();
        Action waitLong = WaitLong.build();
        Action waitMid = MidWait.build();




        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(claw::SetWristCenter),
                        new ParallelAction(
                                arm.upHigh(),
                                arm.outMid(),
                                firstScore
                        ),
                        new ParallelAction(
                                arm.upHigh(),
                                arm.outHigh()
                        ),
                        new ParallelAction(
                                arm.upHigh(),
                                arm.outHigh(),
                                new InstantAction(claw::grabberPlace),
                                waitScore
                        ),
                        new ParallelAction(
                                arm.upHigh(),
                                arm.outRest()
                        ),

                        new ParallelAction(
                                prepGrabSecond,
                                arm.upRest(),
                                arm.outRest(),
                                new InstantAction(claw::grabberStop)
                        ),
                        new ParallelAction(
                                arm.upPick(),
                                arm.outPick(),
                                new InstantAction(claw::grabberPick),
                                secondGrab
                        ),
                        new ParallelAction(
                                arm.upPick(),
                                arm.outPick(),
                                new InstantAction(claw::grabberPick),
                                waitPick
                        ),
                        new ParallelAction(
                                arm.upHigh(),
                                arm.outMid(),
                                secondScore
                        ),
                        new ParallelAction(
                                arm.upHigh(),
                                arm.outHigh()
                        ),
                        new ParallelAction(
                                arm.upHigh(),
                                arm.outMid(),
                                new InstantAction(claw::grabberPlace),
                                waitLong
                        )
                )
        );
    }
}