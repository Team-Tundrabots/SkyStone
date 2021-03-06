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

package org.firstinspires.ftc.teamcode.ops.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.*;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.WebCamera;
import org.firstinspires.ftc.teamcode.components.Ramp;


@TeleOp(name="Game_TeleOp", group="game")
public class Game_TeleOp extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GameTeleBot robot = null;
    private boolean logEnableTrace = false;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new GameTeleBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        //robot.initAll();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.grapple.init();
        robot.ramp.init();
        robot.intake.init();

        robot.logger.logInfo("runOpMode", "===== [ Start TeleOp ]");
        runtime.reset();

        boolean rampUp = false;
        boolean rampDown = false;
        double rampPosition = 1;
        double rampHandicap = 0.075;


        while (opModeIsActive()) {


            /********** Put Your Code Here **********/
            if (robot.driveTrain.isAvailable) {
                double leftX = gamepad1.left_stick_x;
                double leftY = gamepad1.left_stick_y;
                double rightX = gamepad1.right_stick_x;
                double rightY = gamepad1.right_stick_y;

                robot.driveTrain.updateMotorsMechanumDrive(leftX, leftY, rightX, rightY);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Left", "X (%.2f), Y (%.2f)", leftX, leftY);
                telemetry.addData("Right", "X (%.2f), Y (%.2f)", rightX, rightY);

            }

            if (robot.intake.isAvailable) {
                if (gamepad1.left_trigger > 0) {
                    robot.intake.setIntakePower(-1);
                }
                if(gamepad1.left_trigger <= 0)
                {
                    robot.intake.setIntakePower(0);
                }
            }


            if (robot.intake.isAvailable) {
                if (gamepad1.right_trigger > 0) {
                    robot.intake.setIntakePower(0.9);
                }
                if(gamepad1.right_trigger <= 0)
                {
                    robot.intake.setIntakePower(0);
                }
            }

            if(gamepad1.left_bumper){
                rampUp = true;
                rampDown = false;
            }
            if(rampUp){
                rampPosition = rampPosition + 0.25;
                robot.ramp.rampDown(rampPosition);
                robot.ramp.ramp2Down(rampPosition);
                if(rampPosition > 1) {
                    rampUp = false;
                    rampPosition = 1;
                }
            }

            if(gamepad1.right_bumper) {
                rampDown = true;
                rampUp = false;

            }
            if(rampDown){
                if(rampPosition < 0.67) {
                    rampPosition = rampPosition - 0.0008;
                }
                else {
                    rampPosition = rampPosition - 0.005;
                }

                if (rampPosition <= 0.45) {
                    rampPosition = 0.45;
                    rampDown = false;
                }

                robot.ramp.rampDown(rampPosition);
                robot.ramp.ramp2Down(rampPosition);

                /*if(rampPosition <= 0.4) {
                    rampDown = false;
                }*/
            }

            if(gamepad1.x){
                robot.grapple.grappleMoveDown();
            }
            if (gamepad1.y){



                robot.grapple.grappleMoveUp();
            }

            // Show the elapsed game time.
            robot.logger.logInfo("runOpMode", "===== [ TeleOp Complete ] Run Time: %s", runtime.toString());
            telemetry.update();

        }
    }
}