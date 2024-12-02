/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static java.lang.Thread.sleep;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class Drivebase {
    private ElapsedTime runtime = new ElapsedTime();
    private final Supplier<Boolean> opModeIsActive;
    private final Telemetry telemetry;
    /* Declare OpMode members. */

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private CRServo clawRotate;
    private CRServo clawPosition;
    private DcMotor leftSlideExtension;
    private DcMotor rightSlideExtension;
    private DcMotor leftSlideRotate;
    private DcMotor rightSlideRotate;
    private final DcMotor FLDrive;
    private final DcMotor FRDrive;
    private final DcMotor BLDrive;
    private final DcMotor BRDrive;
    private final IMU imu;

    private double headingError = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double SLIDE_SPEED = 1.0;
    private int FLTarget = 0;
    private int FRTarget = 0;
    private int BLTarget = 0;
    private int BRTarget = 0;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public final double MID_SERVO = 0.5;
    public final double HAND_SPEED = 0.02;  // sets rate to move servo
    public final double ARM_UP_POWER = 0.45;
    public final double ARM_DOWN_POWER = -0.45;
    // TODO: change this number once we do encoder math for the correct number.
    static final double COUNTS_PER_MOTOR_REV_312RPM = (1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 11.0)) * 28.0; //Originated from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double COUNTS_PER_MOTOR_REV_223RPM = (1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 11.0)) * 28.0; //Originated from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
    static final double COUNTS_PER_MOTOR_REV_60RPM = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * (1.0 + (46.0 / 11.0)) * 28.0); //Originated from https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-99-5-1-ratio-24mm-length-8mm-rex-shaft-60-rpm-3-3-5v-encoder/
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double SLIDE_GEAR_REDUCTION = 1.0;
    static final double ROTATE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4;   //mm to inches.
    static final double BELT_WHEEL_DIAMETER_INCHES = 38.2 / 25.4; //mm to inches. //Originated from https://www.gobilda.com/2mm-pitch-gt2-hub-mount-timing-belt-pulley-14mm-bore-60-tooth/.
    public final static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_312RPM * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH_SLIDES = (COUNTS_PER_MOTOR_REV_312RPM * SLIDE_GEAR_REDUCTION) / (BELT_WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_DEGREE_ROTATE = (COUNTS_PER_MOTOR_REV_60RPM * ROTATE_GEAR_REDUCTION) / 360;
    static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.2;     // Max turn speed to limit turn rate.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    public final static double ENCODER_PER_INCH = 1.0;

    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Drivebase(HardwareMap hardwareMap, Supplier<Boolean> opModeIsActive, Telemetry telemetry) {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        FLDrive = hardwareMap.dcMotor.get("FLDrive");
        FRDrive = hardwareMap.dcMotor.get("FRDrive");
        BLDrive = hardwareMap.dcMotor.get("BLDrive");
        BRDrive = hardwareMap.dcMotor.get("BRDrive");

        leftSlideExtension = hardwareMap.dcMotor.get("leftSlideExtension");
        rightSlideExtension = hardwareMap.dcMotor.get("rightSlideExtension");
        leftSlideRotate = hardwareMap.dcMotor.get("leftSlideRotate");
        rightSlideRotate = hardwareMap.dcMotor.get("rightSlideRotate");

        // Define and initialize ALL installed servos.
        clawRotate = hardwareMap.get(CRServo.class, "clawRotate");
        clawPosition = hardwareMap.crservo.get("clawPosition");

        //2 motors for rotation of slides. 2 for extension of slides.
        //For claw, 1 servo for rotation(rotates like swerve) and one for opening and closing.
        //right arm stick = extension. Left arm motor = rotation, gamepad = 2.

        imu = hardwareMap.get(IMU.class, "IMU");
        this.telemetry = telemetry;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        this.opModeIsActive = opModeIsActive;

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        leftSlideExtension.setDirection(DcMotor.Direction.FORWARD);
        rightSlideExtension.setDirection(DcMotor.Direction.REVERSE);
        leftSlideRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideRotate.setDirection(DcMotorSimple.Direction.FORWARD);
        clawRotate.setDirection(CRServo.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlideExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * For use in teleop, controlled by a controller.
     *
     * @param axial   Driving input. Negative is back, positive is forward. [-1, 1].
     * @param lateral Strafing input. Negative is left, positive is right. [-1, 1].
     * @param yaw     Turning input. Negative is ccw, positive is clockwise. [-1, 1].
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        double max;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        FLDrive.setPower(leftFrontPower);
        FRDrive.setPower(rightFrontPower);
        BLDrive.setPower(leftBackPower);
        BRDrive.setPower(rightBackPower);
    }

    public void autoMoveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        FLDrive.setPower(leftSpeed);
        FRDrive.setPower(rightSpeed);
        BLDrive.setPower(leftSpeed);
        BRDrive.setPower(rightSpeed);

        sendTelemetry(true);
    }

    /**
     * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive.get()) {

            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            FLTarget = FLDrive.getCurrentPosition() + moveCounts;
            FRTarget = FRDrive.getCurrentPosition() + moveCounts;
            BLTarget = BLDrive.getCurrentPosition() + moveCounts;
            BRTarget = BRDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            FLDrive.setTargetPosition(FLTarget);
            FRDrive.setTargetPosition(FRTarget);
            BLDrive.setTargetPosition(BLTarget);
            BRDrive.setTargetPosition(BRTarget);

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            autoMoveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive.get() &&
                    (FLDrive.isBusy() && FRDrive.isBusy()) && BLDrive.isBusy() && BRDrive.isBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                autoMoveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            autoMoveRobot(0, 0);
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveSideways(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive.get()) {

            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            FLTarget = FLDrive.getCurrentPosition() + moveCounts;
            FRTarget = FRDrive.getCurrentPosition() - moveCounts;
            BLTarget = BLDrive.getCurrentPosition() - moveCounts;
            BRTarget = BRDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            FLDrive.setTargetPosition(FLTarget);
            FRDrive.setTargetPosition(FRTarget);
            BLDrive.setTargetPosition(BLTarget);
            BRDrive.setTargetPosition(BRTarget);

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            autoMoveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive.get() &&
                    (FLDrive.isBusy() && FRDrive.isBusy()) && BLDrive.isBusy() && BRDrive.isBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                autoMoveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            autoMoveRobot(0, 0);
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveSlower(double axial, double lateral, double yaw) {
        driveRobot(axial * .2, lateral * .2, yaw * .2);
    }

    /**
     * Spin on the central axis to point in a new direction.
     * <p>
     * Move will stop if either of these conditions occur:
     * <p>
     * 1) Move gets to the heading (angle)
     * <p>
     * 2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive.get() && (Math.abs(headingError) > HEADING_THRESHOLD) && telemetry != null) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            autoMoveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
        }

        // Stop all motion;
        autoMoveRobot(0, 0);
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive.get() && (holdTimer.time() < holdTime) && telemetry != null) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            autoMoveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
        }

        // Stop all motion;
        autoMoveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void teleOpSlideDrive(double power) {
        //moving slide code.
        if (power > 0.05 || power < -0.05) {
            //set upper limit.
            //check if you've reached the upper limit.
            if (leftSlideExtension.getCurrentPosition() > -3900 || rightSlideExtension.getCurrentPosition() > -3900) {
                //if upper limit's been reached and person is trying to bring slides in let them.
                if (power < 0) {
                    leftSlideExtension.setPower(power);
                    rightSlideExtension.setPower(power);
                }
                //upper limit's been reached and person is trying to bring slides out further so stop them.
                else {
                    leftSlideExtension.setPower(0);
                    rightSlideExtension.setPower(0);
                }
            }
            //set lower limit.
            //check if you've reached the lower limit.
            else if (leftSlideExtension.getCurrentPosition() >= 0 || rightSlideExtension.getCurrentPosition() >= 0) {
                //if slides have reached their lower limit and person's trying to bring slides out let them.
                if (power > 0) {
                    leftSlideExtension.setPower(power);
                    rightSlideExtension.setPower(power);
                }
                //limit's been reached and person is trying to bring slides in further so stop them.
                else {
                    leftSlideExtension.setPower(0);
                    rightSlideExtension.setPower(0);
                }
            }
            //neither limit has been reached so slides can go in or out.
            else {
                leftSlideExtension.setPower(power);
                rightSlideExtension.setPower(power);
            }
        }
        //no joystick power applied.
        else {
            leftSlideExtension.setPower(0);
            rightSlideExtension.setPower(0);
        }
        sendTelemetry(true);
    }

    public void autoEncoderSlide(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive.get()) {

            leftSlideExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftSlideExtension.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_SLIDES);
            newRightTarget = rightSlideExtension.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_SLIDES);
            leftSlideExtension.setTargetPosition(newLeftTarget);
            rightSlideExtension.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftSlideExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftSlideExtension.setPower(Math.abs(speed));
            rightSlideExtension.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive.get() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftSlideExtension.isBusy() && rightSlideExtension.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftSlideExtension.getCurrentPosition(), rightSlideExtension.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftSlideExtension.setPower(0);
            rightSlideExtension.setPower(0);

            // Turn off RUN_TO_POSITION
            leftSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getSlideRotationAngle() {
        double armAngleEncodersAvg = (leftSlideRotate.getCurrentPosition() + rightSlideRotate.getCurrentPosition()) / 2.0;
        return armAngleEncodersAvg / COUNTS_PER_DEGREE_ROTATE;
    }

    public double getSlideExtensionInches() {
        double curExtensionEncoders = (leftSlideExtension.getCurrentPosition() + rightSlideExtension.getCurrentPosition()) / 2.0;
        return curExtensionEncoders / COUNTS_PER_INCH_SLIDES;
    }

    public void teleOpSlideWithLimit(double power) {
        double tolerance = 4.0; //4 inches of tolerance
        double maxFootprint = 42.0 - tolerance; //inches. //Max distance to extend horizontally. //Canhe
        double compressedArmLength = 10.5; //inches //Length of slides when fully in.
        double offsetFromRear = 6.75; //inches //Distance from rear to point of rotation.
        double maxExtension = (maxFootprint - offsetFromRear) / Math.cos(Math.toRadians(getSlideRotationAngle())) - compressedArmLength; //max that can be extended.

        if (getSlideExtensionInches() < maxExtension) { // if slides are under the limit.
            leftSlideExtension.setPower(power);
            rightSlideExtension.setPower(power);
        } else { //slides are over limit so bring them in automatically.
            leftSlideExtension.setPower(-0.50);
            rightSlideExtension.setPower(-0.50);
        }
    }

    public void teleOpSlideRotate(double power) {
        double deadzone = 0.05;

        if (Math.abs(power) < deadzone) { // if the power applied is greater than the minimum power required to move the motor
            power = 0.0;
        }

        double lowerLimit = 5.0;
        double upperLimit = 45.0;
        
        boolean underLowerLimit = getSlideRotationAngle() < lowerLimit;
        boolean aboveUpperLimit = getSlideRotationAngle() > upperLimit;

        if (underLowerLimit && power < 0 || aboveUpperLimit && power > 0) {
            leftSlideRotate.setPower(0.0);
            rightSlideRotate.setPower(0.0);
        } else {
            leftSlideRotate.setPower(power);
            rightSlideRotate.setPower(power);
        }

        sendTelemetry(true);
    }

    public void autoEncoderSlideRotate(double speed,
                                       double leftDegree, double rightDegree,
                                       double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive.get()) {

            leftSlideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftSlideRotate.getCurrentPosition() + (int) (leftDegree * COUNTS_PER_DEGREE_ROTATE);
            newRightTarget = rightSlideRotate.getCurrentPosition() + (int) (rightDegree * COUNTS_PER_DEGREE_ROTATE);
            leftSlideRotate.setTargetPosition(newLeftTarget);
            rightSlideRotate.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftSlideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftSlideRotate.setPower(Math.abs(speed));
            rightSlideRotate.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive.get() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftSlideRotate.isBusy() && rightSlideRotate.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftSlideRotate.getCurrentPosition(), rightSlideRotate.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftSlideRotate.setPower(0);
            rightSlideRotate.setPower(0);

            // Turn off RUN_TO_POSITION
            leftSlideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void closeClaw() {
        {
            clawPosition.setPower(1);
        }
    }

    public void openClaw() {
        {
            clawPosition.setPower(0);
        }
    }

    public void rotateClawLeft() {
        {
            clawRotate.setPower(1);

        }
    }

    public void rotateClawRight() {
        {
            clawRotate.setPower(-1);
        }
    }

    public void rotateClawStop() {
        {
            clawRotate.setPower(0);
        }
    }


    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    public void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d:%7d:%7d", FLTarget, FRTarget, BLTarget, BRTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d:%7d:%7d", FLDrive.getCurrentPosition(), FRDrive.getCurrentPosition(), BLDrive.getCurrentPosition(), BRDrive.getCurrentPosition());
            telemetry.addData("COUNTS_PER_INCH: ", COUNTS_PER_INCH);
            telemetry.addData("COUNTS_PER_MOTOR_REV: ", COUNTS_PER_MOTOR_REV_312RPM);

        } else {
            telemetry.addData("Motion", "Turning");
        }


        telemetry.addData("Arm Rotate Degrees", getSlideRotationAngle());
        telemetry.addData("Slide Extension Inch", getSlideExtensionInches());

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.addData("CurEncoders:", "%7d :%7d", leftSlideExtension.getCurrentPosition(), rightSlideExtension.getCurrentPosition());
        telemetry.addData("Arm Rotate position", "%7d :%7d", leftSlideRotate.getCurrentPosition(), rightSlideRotate.getCurrentPosition());
        telemetry.addData("claw rotate and claw open power", "%7f :%7f", clawRotate.getPower(), clawPosition.getPower());
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
