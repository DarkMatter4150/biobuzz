package org.firstinspires.ftc.teamcode.robot.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.rtpAxon.RTPAxon;

import java.util.Arrays;
import java.util.List;

public class Hardware {
    public GoBildaPinpointDriver pinpoint; //they told me to (the robot error did)
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;
    public List<DcMotor> driveMotors;
    public DcMotorEx outtakeL;
    //public AnalogInput outtakeLEncoder;
    public DcMotorEx outtakeR;
    //public AnalogInput outtakeREncoder;
//    public List<DcMotor> outtakeMotors;
    public Servo outtakeGate;
    public CRServo outtakeRotatorR;
    public AnalogInput outtakeRotatorREncoder;
    public CRServo outtakeRotatorL;
    public AnalogInput outtakeRotatorLEncoder;
    //public RTPAxon outtakeRotatorRAxon;
    public Servo outtakeHood;
    public AnalogInput outtakeHoodEncoder;
    public DcMotor intakeL;
    public  DcMotor intakeR;
    //private Telemetry telemetry;

    public Hardware(HardwareMap hardwareMap) {

        // DRIVE MOTORS
        fR = hardwareMap.get(DcMotor.class, "fR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        driveMotors = Arrays.asList(fR, fL, bR, bL);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // INTAKE SYSTEM
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");

        // OUTTAKE SYSTEM
        outtakeGate = hardwareMap.get(Servo.class, "outtakeGate");

        outtakeRotatorL = hardwareMap.get(CRServo.class, "outtakeRotatorL");
        outtakeRotatorLEncoder = hardwareMap.get(AnalogInput.class, "outtakeRotatorLEncoder");

        outtakeRotatorR = hardwareMap.get(CRServo.class, "outtakeRotatorR");
        outtakeRotatorREncoder = hardwareMap.get(AnalogInput.class, "outtakeRotatorREncoder");
        //outtakeRotatorRAxon = new RTPAxon(outtakeRotatorR, outtakeRotatorREncoder);

        outtakeHood = hardwareMap.get(Servo.class, "outtakeHood");
        outtakeHoodEncoder = hardwareMap.get(AnalogInput.class, "outtakeLauncherEncoder");

        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //outtakeLEncoder = hardwareMap.get(AnalogInput.class, "outtakeLEncoder");

        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");
        outtakeR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //outtakeLEncoder = hardwareMap.get(AnalogInput.class, "outtakeLEncoder");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);

        outtakeL.setDirection(DcMotor.Direction.FORWARD);
        outtakeR.setDirection(DcMotor.Direction.FORWARD);

        intakeL.setDirection(DcMotor.Direction.FORWARD);
        intakeR.setDirection(DcMotor.Direction.REVERSE);

        pinpoint.resetPosAndIMU();

    }

    public void moveForward(double power) {
        bL.setPower(power);
        bR.setPower(power);
        fL.setPower(power);
        fR.setPower(power);
    }

    public void moveBackward(double power) {
        bL.setPower(-power);
        bR.setPower(-power);
        fL.setPower(-power);
        fR.setPower(-power);
    }

    public void strafeLeft(double power) {
        bL.setPower(-power);
        bR.setPower(power);
        fL.setPower(-power);
        fR.setPower(power);
    }

    public void strafeRight(double power) {
        bL.setPower(power);
        bR.setPower(-power);
        fL.setPower(power);
        fR.setPower(-power);
    }

    public final void rotateLeft(double power) {
        bL.setPower(power);
        bR.setPower(power);
        fL.setPower(-power);
        fR.setPower(-power);
    }

    public final void rotateRight(double power) {
        fR.setPower(-power);
        bR.setPower(-power);
        fL.setPower(power);
        bL.setPower(power);
    }

    public final void feedOuttake(double power) {
        //outtakeGate.setPower(power);
    }
    //NEGATIVE IS INTAKE

    public final void rotateOuttake(double power){
        outtakeRotatorL.setPower(power);
        outtakeRotatorR.setPower(power);
        //MUST TEST - I don't know which direction is which
    }

    public final void runOuttake(int rpm) {
        double vel = (rpm/60.0)*28;
        outtakeL.setPower(vel);
        outtakeR.setPower(-vel);
    }

    public final void runOuttake() {
        outtakeL.setPower(1);
        outtakeR.setPower(-1);
    }

    public final void stopOuttake() {
        outtakeL.setPower(0);
        outtakeR.setPower(0);
    }

    public final void stopDrive() {
        fR.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        bL.setPower(0);
    }

    public final void printEncoders(Telemetry telemetry) {
        try {
            telemetry.addData("hood position: ", outtakeHoodEncoder.getVoltage());
        } catch (Exception e) {
            telemetry.addData("hood position: ", "ERROR");
        }
        try {
            //telemetry.addData("turret position: ", outtakeRotatorRAxon.getTotalRotation());
        } catch (Exception e) {
            telemetry.addData("turret position: ", "ERROR");
        }
        try {
            telemetry.addData("outtake speed: ", outtakeL.getVelocity()); //tops out at like 2300
        } catch (Exception e) {
            telemetry.addData("outtake speed: ", "ERROR");
        }

        //telemetry.addData("turret position: ", outtakeRotatorREncoder.getVoltage());
        //telemetry.addData("outtake speed: ", outtakeL.getVelocity());
        //telemetry.addData("status? ", 1);
        telemetry.update();
    }
}
