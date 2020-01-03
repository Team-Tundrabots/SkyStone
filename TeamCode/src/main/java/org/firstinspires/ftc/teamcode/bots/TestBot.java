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

package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.BotComponent;
import org.firstinspires.ftc.teamcode.components.Grapple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.components.GyroNavigator;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Ramp;
import org.firstinspires.ftc.teamcode.components.SkystoneFinder;
import org.firstinspires.ftc.teamcode.components.WebCamera;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.Logger;
import org.firstinspires.ftc.teamcode.components.WebCamNavigator;

public class TestBot extends Bot {

    /* BotComponents */

    public Logger logger = null;
    public WebCamera webCamera = null;
    public DriveTrain driveTrain = null;
    public GyroNavigator gyroNavigator = null;
//    public GyroNavigator gyroNavigator2 = null;
    public WebCamNavigator webCamNavigator = null;
    public Grapple grapple = null;
    //public ColorSensor colorDetection = null;
    public SkystoneFinder skystoneFinder = null;
    public Intake intake = null;
    public Ramp ramp = null;

    /* Constructor */
    public TestBot() {

    }

    public TestBot(OpMode aOpMode) {
        this(aOpMode, false, false);
    }

    public TestBot(OpMode aOpMode, boolean enableTrace, boolean enableTelemetry) {

        logger = new Logger("TestBot", aOpMode, enableTrace, enableTelemetry);
        gyroNavigator = new GyroNavigator(logger, aOpMode);
//        gyroNavigator2 = new GyroNavigator(logger, aOpMode, "imu 1");
        driveTrain = new DriveTrain(logger, aOpMode, "frontLeftMotor", "frontRightMotor",
                "backLeftMotor", "backRightMotor",
                gyroNavigator);

        webCamera = new WebCamera(logger, aOpMode, "Webcam 1");
        webCamNavigator = new WebCamNavigator(logger, aOpMode, webCamera);
        grapple = new Grapple(logger, aOpMode, "servo1", "servo2");
        //colorDetection = new ColorDetection();
        skystoneFinder = new SkystoneFinder(logger, aOpMode);
        intake = new Intake(logger, aOpMode, "Right_Intake", "Left_Intake");
        ramp = new Ramp(logger, aOpMode, "rampServo", "rampServo2");


    }


    public void initAll() {
        gyroNavigator.init();
        driveTrain.init(DriveTrain.InitType.INIT_4WD);
        webCamera.init(WebCamera.InitType.INIT_FOR_FIND_GOLD);
        skystoneFinder.init();
        // webCamNavigator.init();
    }

}

