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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
public class Intake extends BotComponent {

    public DcMotor Right_Intake = null;
    public DcMotor Left_Intake = null;

    public enum InitType {
        INIT_RIGHT_INTAKE,
        INIT_LEFT_INTAKE,
        INIT_4WD
    }

    private boolean rightIntakeEnabled = true;
    private boolean leftIntakeEnabled = true;

public Intake(){
}

    public Intake(Logger aLogger, OpMode aOpMode,
                  String rightIntakeName, String leftIntakeName) {
            super (aLogger, aOpMode);

        Right_Intake = initMotor(rightIntakeName, DcMotor.Direction.REVERSE);
        Left_Intake = initMotor(leftIntakeName, DcMotor.Direction.REVERSE);

    }

    public void init() {
        init(DriveTrain.InitType.INIT_4WD);
    }

    public void init(DriveTrain.InitType initType) {

        switch (initType) {

            case INIT_FRONT_MOTORS:
                rightIntakeEnabled = (Right_Intake != null);
                isAvailable = rightIntakeEnabled;
                break;

            case INIT_BACK_MOTORS:
                leftIntakeEnabled = (Left_Intake != null);
                isAvailable = leftIntakeEnabled;
                break;

            case INIT_4WD:
                rightIntakeEnabled = (Right_Intake != null);
                leftIntakeEnabled = (Left_Intake != null);
                isAvailable = rightIntakeEnabled && leftIntakeEnabled;
                break;
        }

    }

    public void setRightIntakePower(double power){
        if (rightIntakeEnabled) {
            Right_Intake.setPower(power);
        }
    }

    public void setLeftIntakePower(double power){
        if (leftIntakeEnabled) {
            Left_Intake.setPower(power);
        }
    }

    public void setIntakePower (double power){
        if (leftIntakeEnabled) {
            setLeftIntakePower(-power);
        }
        if (rightIntakeEnabled) {
            setRightIntakePower(power);
        }
    }




}

