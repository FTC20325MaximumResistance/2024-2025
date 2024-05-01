package org.firstinspires.ftc.teamcode.myUtil;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.sql.Time;

public class Hardware {
    OpMode opMode;
    public DcMotor frm, flm, blm, brm, arm,lifter,lift;
    public Servo crossbow, claw, wrist, secure;
    public CRServo kraken;
    public BNO055IMU imu;


    public BNO055IMU.Parameters parameters;


   public DcMotor[] drive;

    public void initRobot(OpMode opMode){
        this.opMode = opMode;
        initHardware();
    }
    public void initHardware(){
        try{
            frm = opMode.hardwareMap.dcMotor.get("frm");
            flm = opMode.hardwareMap.dcMotor.get("flm");
            brm = opMode.hardwareMap.dcMotor.get("brm");
            blm = opMode.hardwareMap.dcMotor.get("blm");
            drive = new DcMotor[]{frm, flm, brm, blm};
            for (DcMotor motor: drive){
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
//            frm.setDirection(DcMotorSimple.Direction.REVERSE);
//            flm.setDirection(DcMotorSimple.Direction.REVERSE);
            blm.setDirection(DcMotorSimple.Direction.REVERSE);

//            brm.setDirection(DcMotorSimple.Direction.REVERSE);

        }catch (Exception e) {
            opMode.telemetry.addLine("Drive motors are uninitiated, the robot will not drive forward or backwards");
        }
        try{
            crossbow = opMode.hardwareMap.servo.get("crossbow");
            secure = opMode.hardwareMap.servo.get("secure");
            crossbow.setPosition(1);
            secure.setPosition(0);
//            garret = opMode.hardwareMap.dcMotor.get("garret");
//            garret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            garretG = new Garret(this,opMode);
        } catch (Exception e) {
            opMode.telemetry.addLine("Crossbow servo is not initialized");
        }
        try {
            parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

            imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }catch (Exception e) {
            opMode.telemetry.addLine("IMU not initialized");
        }
        try {
            arm = opMode.hardwareMap.dcMotor.get("arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            claw = opMode.hardwareMap.servo.get("claw");
            //Open: 1 Close: 0
            claw.setPosition(0);
            wrist = opMode.hardwareMap.servo.get("wrist");
            //Ground: 0 Board: 1
            wrist.setPosition(0);
        }catch (Exception e){
            opMode.telemetry.addLine("Arm not initialized");
        }
        try{
            lift = opMode.hardwareMap.dcMotor.get("lift");
            lifter = opMode.hardwareMap.dcMotor.get("lifter");
        }catch (Exception e){
            opMode.telemetry.addLine("Lift not initialized");
        }


        opMode.telemetry.update();
//        crossbow.setPosition(0);
    }



    public void setMotorPowers(double power){
        frm.setPower(power);
        flm.setPower(power);
        blm.setPower(power);
        brm.setPower(power);
    }

    /**
     *
     * @param power1 Front Right
     * @param power2 Front Left
     * @param power3 Back Left
     * @param power4 Back Right
     */
    public void setMotorPowers(double power1, double power2, double power3, double power4){
        frm.setPower(power1);
        flm.setPower(power2);
        blm.setPower(power3);
        brm.setPower(power4);
    }

    public void setMotorTicks(int ticks){
        frm.setTargetPosition(ticks);
        flm.setTargetPosition(ticks);
        blm.setTargetPosition(ticks);
        brm.setTargetPosition(ticks);
    }
    public void setDriveMode(DcMotor.RunMode mode){
        frm.setMode(mode);
        flm.setMode(mode);
        blm.setMode(mode);
        brm.setMode(mode);
    }

    public boolean getTolerance(double val1, double val2, double tolerance){

        return (val1 - tolerance < val2) && (val1 + tolerance >val2);
    }

    public enum directions{
        LEFT,
        RIGHT
    }
    public void waiter(int time) {
        ElapsedTime Time = new ElapsedTime();
        Time.reset();
        while (true) {
            if (Time.milliseconds() > time) {
                break;
            }
        }
    }
}
