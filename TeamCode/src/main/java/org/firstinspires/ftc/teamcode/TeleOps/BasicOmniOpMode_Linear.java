/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOps;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.DataLogging.BatteryDatalogger;
import org.firstinspires.ftc.teamcode.Hardware.Drivebase;
import org.firstinspires.ftc.teamcode.Utility.Config;

@TeleOp(name = "Drive", group = "Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.

    public Servo clipServo = null;
    public Servo wristServo = null;
    public Servo elbowServo = null;
    private BatteryDatalogger.Datalog datalog;
    private VoltageSensor battery;
    private int loopCounterVar = 1;

    @Override
    public void runOpMode() {
        Drivebase driveBase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime timePassed = new ElapsedTime();

        //Datalogging
        // Initialize the datalog
        datalog = new BatteryDatalogger.Datalog("datalog_01");

        // You do not need to fill every field of the datalog
        // every time you call writeLine(); those fields will simply
        // contain the last value.
        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        clipServo = hardwareMap.get(Servo.class, "clipServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");

        clipServo.setPosition(0.1);
        wristServo.setPosition(Config.servoWristMiddlePos);
        //Should be the wrist's lower limit here. //Can go straight from where it rests pointing upwards to 90 degrees. That's what you'll have to do.
        //The servo will be laying down horizontally. Only tune this servo when it's been mounted to the clip and/or robot.
        elbowServo.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            //To switch the neg value given to pos value.
            double slideExtensionJoystick = -gamepad2.right_stick_y;
            double slideRotateJoystick = -gamepad2.left_stick_y;

            //to drive slower.
            boolean driveSlowerGP1 = gamepad1.right_bumper;
            boolean driveSlowerGP2 = gamepad2.left_bumper;

            if (driveSlowerGP1) {
                driveBase.driveRobot(axial * 0.5, lateral * 0.5, yaw * 0.5);
            } else {
                driveBase.driveRobot(axial, lateral, yaw);
            }
            if (driveSlowerGP2) {
                driveBase.teleOpSlideWithLimit(slideExtensionJoystick * 0.2);
                driveBase.teleOpSlideRotate(slideRotateJoystick * 0.2);
            } else {
                driveBase.teleOpSlideWithLimit(slideExtensionJoystick);
                driveBase.teleOpSlideRotate(slideRotateJoystick);
            }

            //open clip
            if (gamepad2.a)
                clipServo.setPosition(0.5);
            //close clip
            if (gamepad2.b) {
                clipServo.setPosition(.1);
            }
            //Rotate clip
            if (gamepad2.dpad_left) {
                wristServo.setPosition(Config.servoWristLeftPos);
            }
            //Rotate clip
            if (gamepad2.dpad_up) {
                wristServo.setPosition(Config.servoWristMiddlePos);
            }
            //Rotate clip
            if (gamepad2.dpad_right) {
                wristServo.setPosition(Config.servoWristRightPos);
            }

            //Pull clip downwards (go 180 degrees,to where the clip's facing straight down).  .
            if (gamepad2.right_trigger > 0.1) {
                elbowServo.setPosition(0.15);
            }
            if (gamepad2.right_bumper) {
                elbowServo.setPosition(0.3);
            }
            //Pulling clip up and down from resting on slides to going straight down. (Where it rests on slides to 180.)
            //Pull clip upwards to 0(back to where it's resting in line with the slides).
            if (gamepad2.left_trigger > 0.1) {
                elbowServo.setPosition(0.8);
            }
        }

        driveBase.dT = timePassed.time();
        timePassed.reset();
        driveBase.totalMotorBatteryConsumption = driveBase.totalMotorBatteryConsumption + (driveBase.LSlideExtensionBatteryConsumption+driveBase.RSlideExtensionBatteryConsumption+driveBase.LSlideRotateBatteryConsumption+driveBase.RSlideRotateBatteryConsumption+driveBase.controlHub.getCurrent(CurrentUnit.MILLIAMPS))*driveBase.dT/3600;
        driveBase.LSlideExtensionBatteryConsumption = driveBase.LSlideExtensionBatteryConsumption + ((DcMotorEx)driveBase.leftSlideExtension).getCurrent(CurrentUnit.MILLIAMPS) * driveBase.dT / 3600;
        driveBase.RSlideExtensionBatteryConsumption = driveBase.RSlideExtensionBatteryConsumption + ((DcMotorEx)driveBase.rightSlideExtension).getCurrent(CurrentUnit.MILLIAMPS) * driveBase.dT / 3600;
        driveBase.LSlideRotateBatteryConsumption = driveBase.LSlideRotateBatteryConsumption + ((DcMotorEx)driveBase.leftSlideRotate).getCurrent(CurrentUnit.MILLIAMPS) * driveBase.dT / 3600;
        driveBase.RSlideRotateBatteryConsumption = driveBase.RSlideRotateBatteryConsumption + ((DcMotorEx)driveBase.rightSlideRotate).getCurrent(CurrentUnit.MILLIAMPS) * driveBase.dT / 3600;

        driveBase.FLDriveBatteryConsumption = driveBase.FLDriveBatteryConsumption + ((DcMotorEx)driveBase.FLDrive).getCurrent(CurrentUnit.MILLIAMPS) * driveBase.dT / 3600;
        driveBase.FRDriveBatteryConsumption = driveBase.FRDriveBatteryConsumption + ((DcMotorEx)driveBase.FRDrive).getCurrent(CurrentUnit.MILLIAMPS) * driveBase.dT / 3600;
        driveBase.BLDriveBatteryConsumption = driveBase.BLDriveBatteryConsumption + ((DcMotorEx)driveBase.BLDrive).getCurrent(CurrentUnit.MILLIAMPS) * driveBase.dT / 3600;
        driveBase.BRDriveBatteryConsumption = driveBase.BRDriveBatteryConsumption + ((DcMotorEx)driveBase.BRDrive).getCurrent(CurrentUnit.MILLIAMPS) * driveBase.dT / 3600;

        driveBase.accessoriesBatteryConsumption = driveBase.accessoriesBatteryConsumption +
                (driveBase.totalMotorBatteryConsumption - driveBase.LSlideExtensionBatteryConsumption - driveBase.RSlideExtensionBatteryConsumption - driveBase.LSlideRotateBatteryConsumption - driveBase.RSlideRotateBatteryConsumption -
                        driveBase.FLDriveBatteryConsumption - driveBase.FRDriveBatteryConsumption - driveBase.BLDriveBatteryConsumption - driveBase.BRDriveBatteryConsumption) * driveBase.dT / 3600;

        telemetry.addData("LSExtensionTarget: ", driveBase.leftSlideExtension.getTargetPosition());
        telemetry.addData("RSExtensionTarget: ", driveBase.rightSlideExtension.getTargetPosition());
        telemetry.addData("LSRotateTarget: ", driveBase.leftSlideRotate.getTargetPosition());
        telemetry.addData("RSRotateTarget: ", driveBase.rightSlideRotate.getTargetPosition());

        telemetry.addData("LSExtensionEncoder: ", driveBase.leftSlideExtension.getCurrentPosition());
        telemetry.addData("RSExtensionEncoder: ", driveBase.rightSlideExtension.getCurrentPosition());
        telemetry.addData("LSRotateEncoder: ", driveBase.leftSlideRotate.getCurrentPosition());
        telemetry.addData("RSRotateEncoder: ", driveBase.rightSlideRotate.getCurrentPosition());

        telemetry.addData("LSExtensionCurrent: ", ((DcMotorEx)driveBase.leftSlideExtension).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RSExtensionCurrent: ", ((DcMotorEx)driveBase.rightSlideExtension).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("LSRotateCurrent: ", ((DcMotorEx)driveBase.leftSlideRotate).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RSRotateCurrent: ", ((DcMotorEx)driveBase.rightSlideRotate).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FLDriveCurrent: ", ((DcMotorEx)driveBase.FLDrive).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FRDriveCurrent: ", ((DcMotorEx)driveBase.FRDrive).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BLDriveCurrent: ", ((DcMotorEx)driveBase.BLDrive).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BRDriveCurrent: ", ((DcMotorEx)driveBase.BRDrive).getCurrent(CurrentUnit.AMPS));

        telemetry.addData("dT: ", driveBase.dT);

        telemetry.addData("LSExtensionBatteryConsumption: ", driveBase.LSlideExtensionBatteryConsumption);
        telemetry.addData("RSExtensionBatteryConsumption: ", driveBase.RSlideExtensionBatteryConsumption);
        telemetry.addData("LSRotateBatteryConsumption: ", driveBase.LSlideRotateBatteryConsumption);
        telemetry.addData("RSRotateBatteryConsumption: ", driveBase.RSlideRotateBatteryConsumption);

        telemetry.addData("FLDriveBatteryConsumption: ", driveBase.FLDriveBatteryConsumption);
        telemetry.addData("FRDriveBatteryConsumption: ", driveBase.FRDriveBatteryConsumption);
        telemetry.addData("BLDriveBatteryConsumption: ", driveBase.BLDriveBatteryConsumption);
        telemetry.addData("BRDriveBatteryConsumption: ", driveBase.BRDriveBatteryConsumption);

        telemetry.addData("totalMotorBatteryConsumption: ", driveBase.totalMotorBatteryConsumption);

        telemetry.addData("accessoriesBatteryConsumption: ", driveBase.accessoriesBatteryConsumption);

        //Datalogging
        datalog.opModeStatus.set("RUNNING");

        datalog.loopCounter.set(loopCounterVar);
        datalog.battery.set(battery.getVoltage());

        datalog.totalMotorBatteryConsumption.set(driveBase.totalMotorBatteryConsumption);
        datalog.LSlideExtensionBatteryConsumption.set(driveBase.LSlideExtensionBatteryConsumption);
        datalog.RSlideExtensionBatteryConsumption.set(driveBase.RSlideExtensionBatteryConsumption); 
        datalog.LSlideRotateBatteryConsumption.set(driveBase.LSlideRotateBatteryConsumption);
        datalog.RSlideRotateBatteryConsumption.set(driveBase.RSlideRotateBatteryConsumption);
        datalog.FLDriveBatteryConsumption.set(driveBase.FLDriveBatteryConsumption);
        datalog.FRDriveBatteryConsumption.set(driveBase.FRDriveBatteryConsumption);
        datalog.BLDriveBatteryConsumption.set(driveBase.BLDriveBatteryConsumption);
        datalog.BRDriveBatteryConsumption.set(driveBase.BRDriveBatteryConsumption);
        datalog.accessoriesBatteryConsumption.set(driveBase.accessoriesBatteryConsumption);
        datalog.BATTERY_CAPACITY.set(driveBase.BATTERY_CAPACITY);

        // The logged timestamp is taken when writeLine() is called.
        datalog.writeLine();

        // Datalog fields are stored as text only; do not format here.
        telemetry.addData("totaltotalMotorBatteryConsumption", datalog.totalMotorBatteryConsumption);
        telemetry.addData("LSlideExtensionBatteryConsumption", datalog.LSlideExtensionBatteryConsumption);
        telemetry.addData("RSlideExtensionBatteryConsumption", datalog.RSlideExtensionBatteryConsumption);
        telemetry.addData("LSlideRotateBatteryConsumption", datalog.LSlideRotateBatteryConsumption);
        telemetry.addData("RSlideRotateBatteryConsumption", datalog.RSlideRotateBatteryConsumption);
        telemetry.addData("FLDriveBatteryConsumption", datalog.FLDriveBatteryConsumption);
        telemetry.addData("FRDriveBatteryConsumption", datalog.FRDriveBatteryConsumption);
        telemetry.addData("BLDriveBatteryConsumption", datalog.BLDriveBatteryConsumption);
        telemetry.addData("BRDriveBatteryConsumption", datalog.BRDriveBatteryConsumption);
        telemetry.addData("accessoriesBatteryConsumption", datalog.accessoriesBatteryConsumption);
        telemetry.addLine();
        telemetry.addData("OpMode Status", datalog.opModeStatus);
        telemetry.addData("Loop Counter", datalog.loopCounter);
        telemetry.addData("Battery", datalog.battery);
        telemetry.addData("batteryCapacity", datalog.BATTERY_CAPACITY);

        telemetry.update();

        sleep(20);

        //Increase by one the amount of times the code has run.
        loopCounterVar++;
    }
}