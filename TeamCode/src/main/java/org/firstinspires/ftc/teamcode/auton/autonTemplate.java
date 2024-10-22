package org.firstinspires.ftc.teamcode.auton;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.compVis;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.visionLib;

//@Config
@Autonomous(name="Auton Template")
public class autonTemplate extends LinearOpMode {
    MecanumHardAuto r = new MecanumHardAuto();
    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);
        waitForStart();
        r.flm.setDirection(DcMotorSimple.Direction.REVERSE);
//        r.moveInches(0.1, 10, Hardware.directions.RIGHT);
        r.claw.setPosition(0.5);
        telemetry.addData("claw: ",r.claw.getPosition());
        telemetry.update();

    }
}
