/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrainSimple extends BotComponent {

    public DcMotor motorFL = null;
    public DcMotor motorFR = null;

    public DcMotor motorBL = null;
    public DcMotor motorBR = null;

    private double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    private double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                             (WHEEL_DIAMETER_INCHES * 3.1415);

    private double MAX_INCHES_PER_SECOND   = 6;

    /* Constructor */
    public DriveTrainSimple() {

    }

    public DriveTrainSimple(Logger aLogger, OpMode aOpMode,
                      String frontLeftMotorName, String frontRightMotorName,
                      String backLeftMotorName, String backRightMotorName)
    {
        super(aLogger, aOpMode);

        motorFL = initMotor(frontLeftMotorName, DcMotor.Direction.REVERSE);
        motorFR = initMotor(frontRightMotorName);

        motorBL = initMotor(backLeftMotorName, DcMotor.Direction.REVERSE);
        motorBR = initMotor(backRightMotorName);
    }

    public void stop() {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    public void updateMotorsMechanumDrive(double leftX, double leftY, double rightX, double rightY) {


        double lX = -leftX;
        double lY = leftY;
        double rX = -rightX;
        double rY = rightY;

        double r = Math.hypot(lX, lY);
        double robotAngle = Math.atan2(lY, lX) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) + rX;
        final double v2 = r * Math.sin(robotAngle) - rX;
        final double v3 = r * Math.sin(robotAngle) + rX;
        final double v4 = r * Math.cos(robotAngle) - rX;

        motorFL.setPower(v1);
        motorFR.setPower(v2);
        motorBL.setPower(v3);
        motorBR.setPower(v4);

        if (!opModeIsActive()) { stop(); }

    }

    private void disableEncoders() {
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetEncoders() {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        logger.logDebug("resetEncoders", "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
        logger.logDebug("resetEncoders", "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());

    }

    private void setMotorTarget(DcMotor motor, double inches) {
        int target = motor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void encoderDrive(double power, double leftInches, double rightInches) {

        double timeoutSeconds = (1 / Math.abs(power)) * MAX_INCHES_PER_SECOND;

        resetEncoders();

        setMotorTarget(motorFL, leftInches);
        setMotorTarget(motorFR, rightInches);
        setMotorTarget(motorBL, leftInches);
        setMotorTarget(motorBR, rightInches);

        ElapsedTime runtime = new ElapsedTime();

        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();

            motorFL.setPower(power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(power);

            logger.setDebugFilter("encoderDrive");

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSeconds) &&
                    (motorFL.isBusy() || motorBR.isBusy()) &&
                    (motorFR.isBusy() || motorBL.isBusy()) )
            {

                logger.logDebug("encoderDrive", "Inches: Left:%f  Right:%f", leftInches, rightInches);
                logger.logDebug("encoderDrive", "Target: FL:%7d   FR:%7d", motorFL.getTargetPosition(), motorFR.getTargetPosition());
                logger.logDebug("encoderDrive", "Target: BL:%7d   BR:%7d", motorBL.getTargetPosition(), motorBR.getTargetPosition());
                logger.logDebug("encoderDrive", "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
                logger.logDebug("encoderDrive", "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
                logger.logDebug("encoderDrive", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

                logger.incrementDebugFilterCount();

                opMode.telemetry.update();
            }

            // Stop all motion;
            stop();

            logger.clearDebugFilter();
            logger.logDebug("encoderDrive", "=== [STOP] ===");
            logger.logDebug("encoderDrive", "Inches: Left:%f  Right:%f", leftInches, rightInches);
            logger.logDebug("encoderDrive", "Target: FL:%7d   FR:%7d", motorFL.getTargetPosition(), motorFR.getTargetPosition());
            logger.logDebug("encoderDrive", "Target: BL:%7d   BR:%7d", motorBL.getTargetPosition(), motorBR.getTargetPosition());
            logger.logDebug("encoderDrive", "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
            logger.logDebug("encoderDrive", "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
            logger.logDebug("encoderDrive", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

            disableEncoders();

        }


    }

    public void encoderDrive(double power, double inches) {
        encoderDrive(power, inches, inches);
    }

    public void crabEncoderLeft(double power, double inches) {
        crabEncoder(power, inches, -1, "crabEncoderLeft");
    }

    public void crabEncoderRight(double power, double inches) {
        crabEncoder(power, inches, 1, "crabEncoderRight");
    }

    private void crabEncoder(double power, double inches, int direction, String funcName) {

        double timeoutSeconds = (1 / Math.abs(power)) * MAX_INCHES_PER_SECOND;

        resetEncoders();

        setMotorTarget(motorFL,  direction * inches);
        setMotorTarget(motorFR,  direction * inches);
        setMotorTarget(motorBL, -1 * direction * inches);
        setMotorTarget(motorBR, -1 * direction * inches);

        ElapsedTime runtime = new ElapsedTime();

        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();

            updateMotorsMechanumDrive(direction * power, 0, 0, -power);

            logger.setDebugFilter(funcName);

            while ( opModeIsActive() && (runtime.seconds() < timeoutSeconds) &&
                    !(motorFL.isBusy() || motorBR.isBusy() || motorFR.isBusy() || motorBL.isBusy()) )

            {

                logger.logDebug(funcName, "Inches: %f", inches);
                logger.logDebug(funcName, "Target: FL:%7d   FR:%7d", motorFL.getTargetPosition(), motorFR.getTargetPosition());
                logger.logDebug(funcName, "Target: BL:%7d   BR:%7d", motorBL.getTargetPosition(), motorBR.getTargetPosition());
                logger.logDebug(funcName, "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
                logger.logDebug(funcName, "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
                logger.logDebug(funcName, "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

                logger.incrementDebugFilterCount();

                opMode.telemetry.update();
            }

            // Stop all motion;
            stop();

            logger.clearDebugFilter();
            logger.logDebug(funcName, "=== [STOP] ===");
            logger.logDebug(funcName, "Inches: %f", inches);
            logger.logDebug(funcName, "Target: FL:%7d   FR:%7d", motorFL.getTargetPosition(), motorFR.getTargetPosition());
            logger.logDebug(funcName, "Target: BL:%7d   BR:%7d", motorBL.getTargetPosition(), motorBR.getTargetPosition());
            logger.logDebug(funcName, "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
            logger.logDebug(funcName, "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
            logger.logDebug(funcName, "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

            disableEncoders();

        }

    }

}

