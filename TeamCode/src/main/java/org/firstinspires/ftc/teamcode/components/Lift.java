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
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
public class Lift extends BotComponent {
    private String LiftName;
    private String ServoName;

    public DcMotor liftMotor = null;
    public Servo liftDrop = null;


    private boolean liftMotorEnabled = true;
    private boolean liftDropEnabled = true;

    public Lift(){

    }

    public Lift(Logger aLogger, OpMode aOpMode,
                String aLiftName, String aServoName) {
            super (aLogger, aOpMode);
        LiftName = aLiftName;
        ServoName = aServoName;



    }

    public void init() {

        //define and initialize motors

        liftMotor = initMotor(LiftName, DcMotor.Direction.REVERSE);
        liftDrop = initServo(ServoName, 0.23);

        if (liftMotor != null) {
            isAvailable = true;
        }
        if(liftDrop != null){
            isAvailable = true;
        }

        logger.logInfo("Intake", "isAvailable: %b", isAvailable);

    }


    public void setLiftPowerUp (){
        liftMotor.setPower(1);
    }

    public void setLiftPowerDown(){
        liftMotor.setPower(-1);
    }

    public void setLiftPowerNull()
    {
        liftMotor.setPower(0);
    }


    public void setServoUp (){
        liftDrop.setPosition(0.82);
    }

    public void setServoDown(){
        liftDrop.setPosition(0.23);
    }




}

