package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
//@Disabled
public class armSubSystemTest extends OpMode {
    private Motor arm;
    private Motor outArm;
    private Motor armEnc;

    double outPower = 0;

    private PIDController armPID,outPID;

    public static int outCurrent;
    public static int armCurrent;

    public static int armOffset;

    public static int ArmTarget = 0;
    public static int OutTarget = 0;

    private final double ticks_in_degree =  5281.1 / 360; //gobilda 30rpm encoder
    private final double ticks_in_degree_rev =  8192 / 360; //rev thru bore

    private final double motor_to_rev_ratio = 5281.1/8192;

    public static double arm_p = 0.0125, arm_i = 0.06, arm_d = 0.00075;
    public static double arm_f = 0.08;

    public static double out_p = 0.004, out_i = 0, out_d = 0;

    public static int ReqArmTarget = 500, ReqOutTarget = 0;

    double armInitDelay,PrevArmPos;
    boolean arminitialized;

    @Override
    public void init(){
        arminitialized = false;
        armInitDelay = 0;
        arm = new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312);
        outArm = new Motor(hardwareMap, "outArm", Motor.GoBILDA.RPM_312);
        armEnc = new Motor(hardwareMap, "armEnc", Motor.GoBILDA.RPM_312);

        arm.setInverted(false);
        armEnc.setInverted(true);
        outArm.setInverted(true);
        outArm.resetEncoder();
        arm.resetEncoder();
        armEnc.resetEncoder();
        armOffset = 0;
        arminitialized = false;

        armPID = new PIDController(arm_p,arm_i,arm_d);
        outPID = new PIDController(out_p,out_i,out_d);
    }


    @Override
    public void loop()  {

        armCurrent = arm.getCurrentPosition();
        outCurrent = outArm.getCurrentPosition();
        if (!arminitialized){
            if ((PrevArmPos > armCurrent-5) && (PrevArmPos < armCurrent+5)) {
                if(armInitDelay > 10){
                    armOffset = armCurrent - (int)(armEnc.getCurrentPosition()*motor_to_rev_ratio);
                    arminitialized = true;
                }
                else {armInitDelay++;}
            }
            else{
                PrevArmPos = armCurrent;
                armInitDelay = 0;
            }
        }


        if(outCurrent < 300) {
            ArmTarget = ReqArmTarget+armOffset;
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


        //outArm.setTargetPosition(OutTarget);
        telemetry.addData("pos", armCurrent);
        telemetry.addData("target", ArmTarget);

        telemetry.addData("power", power);

        telemetry.addData("outPower", outpid);


        telemetry.update();


    }
}

