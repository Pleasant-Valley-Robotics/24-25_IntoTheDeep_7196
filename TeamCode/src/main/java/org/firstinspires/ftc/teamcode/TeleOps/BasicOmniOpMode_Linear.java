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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;
import org.firstinspires.ftc.teamcode.Utility.Config;



/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Drive", group = "Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.

    public Servo leftClaw = null;
    public Servo rightHand = null;
    public Servo clawWrist = null;
    double clawOffset = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Drivebase driveBase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        leftClaw = hardwareMap.get(Servo.class, "left_hand");
        rightHand = hardwareMap.get(Servo.class, "right_hand");
        clawWrist = hardwareMap.get(Servo.class, "wristServo");

        leftClaw.setPosition(0.1);
        rightHand.setPosition(Config.servoWristLeftPos);
        //Should be the wrist's lower limit here. //Can go straight from where it rests pointing upwards to 90 degrees. That's what you'll have to do.
        //The servo will be laying down horizontally. Only tune this servo when it's been mounted to the clip and/or robot.
        clawWrist.setPosition(.6);

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
            boolean driveSlowerGP2 = gamepad1.left_bumper;

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
                leftClaw.setPosition(0.5);
            //close clip
            if (gamepad2.b) {
                leftClaw.setPosition(.1);
            }
            if (gamepad2.dpad_left) {
                rightHand.setPosition(Config.servoWristLeftPos);
            }
            if (gamepad2.dpad_up) {
                rightHand.setPosition(Config.servoWristMiddlePos);
            }
            if (gamepad2.dpad_right) {
                rightHand.setPosition(Config.servoWristRightPos);
            }
        }
    }
}