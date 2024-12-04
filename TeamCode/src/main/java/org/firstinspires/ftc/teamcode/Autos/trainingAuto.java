package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.SignificantMotionDetection;
import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@TeleOp(name = "TestingServoTeleOp")
public class trainingAuto extends LinearOpMode {
    private double SIDE_SPEED = .2;
    private Servo clawPosition;

    public void runOpMode() {
        //Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();
        clawPosition = hardwareMap.servo.get("clawPosition");

        clawPosition.setPosition(0);
        clawPosition.setPosition(.3);


        //Put code here
        //Set manually for now with number.

    }
}
