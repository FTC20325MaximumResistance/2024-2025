package org.firstinspires.ftc.teamcode.auton;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.compVis;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.visionLib;
import org.firstinspires.ftc.teamcode.myUtil.threads.auto.Arm;

//@Config
@Autonomous(name="Auton Template")
public class autonTemplate extends LinearOpMode {
    public static boolean L = false;
    public static boolean R = false;
    public static boolean M = false;
    MecanumHardAuto r = new MecanumHardAuto();
    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);


        String loc = L ? "Left" : R ? "Right" : "Mid";
        telemetry.addData("Location", loc);
        telemetry.update();
        waitForStart();
        telemetry.addLine("you are running a template!!");

        switch (loc){
            default:
                // add default case
                break;
            case "Left":
                // add left case
                break;
            case "Right":
                //add right case
                break;
        }

    }
}
