package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.System_Constants.SpecimenConstants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Official RED RIGHT Auto", group = "Autonomous")

public class OfficialRedRightAuto extends LinearOpMode {


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

        SpecimenSubsystem specimen = new SpecimenSubsystem(
                new Motor(hardwareMap, "leftClimb", Motor.GoBILDA.RPM_30),
                new Motor(hardwareMap, "rightClimb", Motor.GoBILDA.RPM_30),
                new SimpleServo(hardwareMap, "leftPick", 0,1),
                new SimpleServo(hardwareMap, "rightPick", 0,1));

        Pose2d initialPose = new Pose2d(-12, 62, Math.toRadians(-90));
        Pose2d startScorePos = new Pose2d(0, 36, Math.toRadians(180));
        Pose2d finishScorePos = new Pose2d(0, 30, Math.toRadians(180));
        Pose2d clearScorePos = new Pose2d(0, 45, Math.toRadians(180));
        /*Pose2d pickSamplePos = new Pose2d(-8, 42, Math.toRadians(-90));
        Pose2d dispensePos = new Pose2d(-45, 10, Math.toRadians(180));
        Pose2d finishPos = new Pose2d(51, 50, Math.toRadians(50));*/ //TODO: Will need to change if update Trajectory
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder StartFirstScore = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(0, 36), Math.toRadians(180));

        TrajectoryActionBuilder FinishScore = drive.actionBuilder(startScorePos)
                .strafeToSplineHeading(new Vector2d(0, 30), Math.toRadians(180));

        TrajectoryActionBuilder ClearScore = drive.actionBuilder(finishScorePos)
                .strafeToSplineHeading(new Vector2d(0, 45), Math.toRadians(180));

        TrajectoryActionBuilder GetSamples = drive.actionBuilder(clearScorePos)
                .strafeToSplineHeading(new Vector2d(-32, 45), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-32, 12), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-45, 12), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-45, 57), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-45, 12), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-55, 12), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(-55, 57), Math.toRadians(-90));
                //.strafeToSplineHeading(new Vector2d(-55, 36), Math.toRadians(-90));
        /*TrajectoryActionBuilder GetToSample = drive.actionBuilder(finishScorePos)
                .strafeToSplineHeading(new Vector2d(-35, 42), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-30, 10), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-45, 10), Math.toRadians(180));

        TrajectoryActionBuilder Dispense = drive.actionBuilder(dispensePos)
                .strafeToSplineHeading(new Vector2d(-48, 58), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-40, 10), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-54, 10), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-48, 58), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-42, 55), Math.toRadians(-90));*/


       /* TrajectoryActionBuilder WaitPickShort = drive.actionBuilder(pickSamplePos)
                .waitSeconds(0.6);
        TrajectoryActionBuilder WaitReallyLong = drive.actionBuilder(pickSamplePos)
                .waitSeconds(30);*/

        //Start Movements Like Claw/Wrist If Needed

        Action scoreFirst = StartFirstScore.build();
        Action finishScoreFirst = FinishScore.build();
        Action clearScore = ClearScore.build();
        Action getSamples = GetSamples.build();
        /*Action getToSample = GetToSample.build();
        Action dispense = Dispense.build();

        Action waitPickShort = WaitPickShort.build();
        Action waitLong = WaitReallyLong.build();*/



        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(specimen::specimenClawClose),

                        new ParallelAction(
                                specimen.deliverSpecimen(),
                                scoreFirst
                        ),
                        finishScoreFirst,

                        new ParallelAction(
                                new InstantAction(specimen::specimenClawClose),
                                specimen.scoreSpecimen()
                        ) ,
                        new InstantAction(specimen::specimenClawOpen),
                        clearScore,
                        new ParallelAction(
                                specimen.lowerLeftArm(),
                                getSamples
                        )/*,
                        new InstantAction(claw::grabberPlace),
                        new ParallelAction(
                                new InstantAction(claw::SetWristCenter),
                                arm.upRest(),
                                arm.outRest(),
                                getToSample
                        ),
                        new InstantAction(claw::grabberStop),
                        new InstantAction(claw::SetWristCenter),
                        new ParallelAction(
                                //new InstantAction(claw::grabberPick),
                                arm.upRest(),
                                arm.outRest()
                                //waitPickShort
                        ),
                        new ParallelAction(
                                //new InstantAction(claw::grabberStop),
                                arm.upRest(),
                                arm.outRest(),
                                dispense
                        ),
                        new ParallelAction(
                                //new InstantAction(claw::grabberPlace),
                                arm.upZero(),
                                arm.outZero()
                        ),
                        waitLong*/
                )
        );
    }
}