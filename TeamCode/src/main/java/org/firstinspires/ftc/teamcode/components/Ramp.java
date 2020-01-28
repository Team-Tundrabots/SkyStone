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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.components.DriveTrain;

//@Disabled
public class Ramp extends BotComponent {
    private String rampServoName1;
    private String rampServoName2;
    public Servo rampServo1 = null;
    public Servo rampServo2 = null;
    double SERVO_DOWN_POSITION = 0.5;
    double SERVO_UP_POSITION = 1;
public Ramp(){
}
public Ramp(Logger aLogger, OpMode aOpMode, String aRampServoName1, String aRampServoName2){
    super(aLogger, aOpMode);
    rampServoName1 = aRampServoName1;
    rampServoName2 = aRampServoName2;
}

public void init( ){

    logger.logDebug("initservo", "IamWalrus");
    rampServo1 = initServo(rampServoName1, SERVO_UP_POSITION);
    rampServo2 = initServo(rampServoName2, SERVO_UP_POSITION);
    if(rampServo1 != null && rampServo2 != null){
        isAvailable = true;
    }

    logger .logInfo("Grapple","isAvailable: %b",isAvailable);
}

public void rampDown(double x){
    logger.logDebug("servoMoveDown", "walrus");
    rampServo1.setPosition(x);
}

public void ramp2Down(double x){
    rampServo2.setPosition(x);
}

public void rampUp(){
    rampServo1.setPosition(SERVO_UP_POSITION);
}

public void ramp2Up(){
    rampServo2.setPosition(SERVO_UP_POSITION);
}
}

