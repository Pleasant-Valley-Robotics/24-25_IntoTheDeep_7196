package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "ParkingAuto", group = "Auto")
public class ParkingAuto extends LinearOpMode {
    private double SIDE_SPEED = .2;
    private Servo clawPosition;

    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();
        drivebase.driveSideways(1, 20, 0);
    }
}
