package org.firstinspires.ftc.teamcode.ops.gyrotest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.TestBot;
import org.firstinspires.ftc.teamcode.components.DriveTrain;


@Autonomous(name="Time_Auto_Red_Platform", group="gyrotest")
//@Disabled
public class Matthew_Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TestBot robot = null;
    private boolean logEnableTrace = true;
    private boolean logToTelemetry = true;


    @Override
    public void runOpMode() {

        robot = new TestBot(this, logEnableTrace, logToTelemetry);
        robot.logger.logInfo("runOpMode", "===== [ Start Initializing ]");

        /* Use either robot.initAll or select only the components that need initializing below */
        //robot.initAll();
        robot.driveTrain.init(DriveTrain.InitType.INIT_4WD);
        robot.gyroNavigator.init();
        //robot.gyroNavigator2.init();

        robot.logger.logInfo("runOpMode", "===== [ Initialization Complete ]");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.logger.logInfo("runOpMode", "===== [ Start Autonomous ]");
        runtime.reset();

        double angle1 = robot.gyroNavigator.getAngle();
        //   double angle2 = robot.gyroNavigator2.getAngle();

         /*  robot.driveTrain.encoderDrive(1, -0.5);
           robot.driveTrain.gyroRotate(90, 0.5, true, false);
           robot.driveTrain.encoderDrive(1, -40);
           robot.driveTrain.gyroRotate(-90, 0.5, true, false);
           robot.driveTrain.encoderDrive(1, -10);
           robot.driveTrain.encoderDrive(1, 10); */

        robot.driveTrain.moveBackward(.85, .75);

        robot.driveTrain.gyroRotate(-95, .75, true, false);

        robot.driveTrain.moveForward(0.7, -0.5);

        robot.driveTrain.pause(1);

        robot.grapple.servoMoveDown();

        robot.grapple.servo2MoveDown();

        robot.driveTrain.pause(1);

        robot.driveTrain.moveForward(1.25, 0.5);

        //Move because the robot can not fine adjust to make the gyro happy with the platform in tow
        robot.driveTrain.move(1, -1, 1);
        robot.grapple.servoMoveUp();
        robot.grapple.servo2MoveUp();
        robot.driveTrain.moveBackward(.5, .75);
        robot.driveTrain.pause(1);
        robot.driveTrain.pause(1);
        robot.driveTrain.moveForward(0.5,  .75);



        //robot.logger.logInfo("runOpMode", "Angles: 1:%f", angle1);

        //  }

        // Show the elapsed game time.
        robot.logger.logInfo("runOpMode", "===== [ Autonomous Complete ] Run Time: %s", runtime.toString());
        telemetry.update();

    }
}
