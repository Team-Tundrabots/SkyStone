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

import org.firstinspires.ftc.teamcode.components.BotComponent;
import org.firstinspires.ftc.teamcode.components.GyroNavigator;
import org.firstinspires.ftc.teamcode.components.Logger;

public class DriveTrainSimple extends BotComponent {

    private String motorFLName = "frontLeftMotor";
    private String motorFRName = "frontRightMotor";
    private String motorBLName = "backLeftMotor";
    private String motorBRName = "backRightMotor";


    public DcMotor motorFL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBL = null;
    public DcMotor motorBR = null;

    private boolean frontMotorsEnabled = false;
    private boolean backMotorsEnabled   = false;

    private double COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    private double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double MAX_INCHES_PER_SECOND   = 6;

    /* Constructor */
    public DriveTrainSimple() {

    }

    public DriveTrainSimple(Logger aLogger, OpMode aOpMode,
                            String aMotorFLName, String aMotorFRName,
                            String aMotorBLName, String aMotorBRName)
    {
        super(aLogger, aOpMode);
        motorFLName = aMotorFLName;
        motorFRName = aMotorFRName;
        motorBLName = aMotorBLName;
        motorBRName = aMotorBRName;
    }

    public void init() {

        motorFL = initMotor(motorFLName, DcMotor.Direction.REVERSE);
        motorFR = initMotor(motorFRName);

        motorBL = initMotor(motorBLName, DcMotor.Direction.REVERSE);
        motorBR = initMotor(motorBRName);

        frontMotorsEnabled = (motorFL != null) && (motorFR != null);
        backMotorsEnabled = (motorBL != null) && (motorBR != null);
        isAvailable = frontMotorsEnabled && backMotorsEnabled;

        logger.logInfo("DriveTrainSimple","isAvailable: %b", isAvailable);

    }

    private void setPower(double powerFL, double powerFR, double powerBL, double powerBR) {
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBL.setPower(powerBR);
    }

    private void driveByTime(double power, double seconds) {

        ElapsedTime runtime = new ElapsedTime();
        setPower(power, power, power, power);

        runtime.reset();
        while(runtime.seconds() < seconds && opModeIsActive()) {
            opMode.telemetry.addData("Path", "Time: %2.5f S Elapsed", runtime.seconds());
            opMode.telemetry.update();
        }
        stop();

    }

    public void stop() {
        setPower(0, 0, 0, 0);
    }

    public void driveMechanum(double leftX, double leftY, double rightX, double rightY) {


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

    public void driveByEncoder(double power, double inches) {

        double timeoutSeconds = (1 / Math.abs(power)) * MAX_INCHES_PER_SECOND;

        int targetFL = motorFL.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        int targetFR = motorFR.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        int targetBL = motorBL.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        int targetBR = motorBR.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

        int newRightTarget;

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setTargetPosition(targetFL);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setTargetPosition(targetFR);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setTargetPosition(targetBL);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setTargetPosition(targetBR);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        setPower(power, power, power, power);

        logger.setDebugFilter("driveByEncoder");

        while (opModeIsActive() && runtime.seconds() < timeoutSeconds
                && ( motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy() ))

        {

            logger.logDebug("driveByEncoder", "Inches: %f", inches);
            logger.logDebug("driveByEncoder", "Target: Left:%7d Right:%7d", targetFL, targetFR);
            logger.logDebug("driveByEncoder", "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
            logger.logDebug("driveByEncoder", "Target: Left:%7d Right:%7d", targetBL, targetBR);
            logger.logDebug("driveByEncoder", "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
            logger.logDebug("driveByEncoder", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

            logger.incrementDebugFilterCount();
            opMode.telemetry.update();
        }

        // Stop all motion;
        stop();

        logger.clearDebugFilter();
        logger.logDebug("driveByEncoder", "Inches: %f", inches);
        logger.logDebug("driveByEncoder", "Target: Left:%7d Right:%7d", targetFL, targetFR);
        logger.logDebug("driveByEncoder", "Front:  Left:%7d Right:%7d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
        logger.logDebug("driveByEncoder", "Target: Left:%7d Right:%7d", targetBL, targetBR);
        logger.logDebug("driveByEncoder", "Back:   Left:%7d Right:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
        logger.logDebug("driveByEncoder", "runtime.seconds: %f timeout: %f", runtime.seconds(), timeoutSeconds);

        // disable encoders;
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


}

