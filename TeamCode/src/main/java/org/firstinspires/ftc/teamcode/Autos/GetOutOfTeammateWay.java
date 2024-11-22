package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "GetOutOfWay")
public class GetOutOfTeammateWay extends LinearOpMode {
    private double SIDE_SPEED = .2;
    private double DRIVE_SPEED = .9;


    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();

        //Put code here
        drivebase.driveStraight(DRIVE_SPEED, 48, 0);
        drivebase.sendTelemetry(true);
    }
}
