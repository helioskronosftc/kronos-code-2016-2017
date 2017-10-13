/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.KronosHardware;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: blue beacon", group="Pushbot")
//@Disabled
public class bluebeacon extends LinearOpMode {

    /* Declare OpMode members. */
    KronosHardware robot = new KronosHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final String Blue_opMode = "true";
    static final String Red_opMode = "false";

    static final double COUNTS_PER_MOTOR_REV = 1080; //1440 ;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    boolean colorresred = false;
    boolean colorresblue = false;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        long debugpause = 500;
        long debugpause2 = 500;

        robot.launcherMotor.setPower(0.3);
        sleep(500);
        robot.launcherMotor.setPower(0.5);
        sleep(500);
        robot.launcherMotor.setPower(0.6);
        telemetry.addData("launcher", "ready");
        telemetry.update();
        sleep(500);

        // drive backwards to the center vortex
        telemetry.addData("backward", "21");
        telemetry.update();
        backward(15, 0.2, 0);
        backward(5, 0.15, 0); // slow down to help stopping straight

        // shoot 2 balls
        telemetry.addData("launch", "ball 1");
        telemetry.update();
        robot.servo_launch.setPosition(0.23);
        sleep(500);
        robot.servo_launch.setPosition(0.45);
        sleep(2000);
        telemetry.addData("launch", "ball 2");
        telemetry.update();
        robot.servo_launch.setPosition(0.23);
        sleep(500);
        robot.servo_launch.setPosition(0.45);
        sleep(250);

        // turn off the launcher
        robot.launcherMotor.setPower(0);
        telemetry.addData("launcher", "off");
        telemetry.update();

        //sleep(debugpause);
        telemetry.addData("rotate", "120");
        telemetry.update();
        //sleep(debugpause2);
        rotate(120, 0.3, 1);

        //sleep(debugpause);
        telemetry.addData("forward", "31");
        telemetry.update();
        //sleep(debugpause2);
        forward(31, 0.5, 0);

        //sleep(debugpause);
        telemetry.addData("find", "line");
        telemetry.update();
        //sleep(debugpause2);
        linefinder(5000);

        robot.servoL1.setPosition(0.6);

        //sleep(debugpause);
        telemetry.addData("forward", "5");
        telemetry.update();
        //sleep(debugpause2);
        forward(5, 0.3, 0);

        //sleep(debugpause);
        telemetry.addData("rotate", "-35");
        telemetry.update();
        //sleep(debugpause2);
        rotate(22, 0.3, -1);

        //sleep(debugpause);
        telemetry.addData("follow", "line");
        telemetry.update();
        //sleep(debugpause2);
        linefollower(5000);  //  send with timeout in mS
        telemetry.addData("dist2", robot.distanceSensor2.getRawLightDetected());
        telemetry.addData("time", runtime.milliseconds());
        telemetry.update();

        robot.servoL1.setPosition(0.5);

       sleep(debugpause);
        telemetry.addData("check", "color");
        telemetry.update();
       sleep(debugpause2);
        colorcheck(); // check the beacon color on the left side
        //First colorcheck in the program

        //sleep(debugpause);
        telemetry.addData("backward", "4");
        telemetry.update();
        //sleep(debugpause2);
        backward(5, 0.3, 0.0); // back up after pressing beacon

        //sleep(debugpause);
        telemetry.addData("rotate", "90");
        telemetry.update();
        //sleep(debugpause2);
        rotate(87, 0.3, 1);

        //sleep(debugpause);
        telemetry.addData("forward", "34");
        telemetry.update();
        //sleep(debugpause2);
        forward(34, 0.3, 0.0);

        //sleep(debugpause);
        telemetry.addData("find", "2nd line");
        telemetry.update();
        //sleep(debugpause2);
        linefinder(4000);

        robot.servoL1.setPosition(0.6);

//        sleep(debugpause);
        telemetry.addData("forward", "4");
        telemetry.update();
  //      sleep(debugpause2);
        forward(4, 0.3, 0.0);

        robot.servoL1.setPosition(0.6);

    //    sleep(debugpause);
        telemetry.addData("rotate", "-90");
        telemetry.update();
      //  sleep(debugpause2);
        rotate(90, 0.3, -1);


      //  sleep(debugpause);
        telemetry.addData("follow ", "line");
        telemetry.update();
       // sleep(debugpause2);
        linefollower(3000);

        robot.servoL1.setPosition(0.5);

         sleep(debugpause);
        telemetry.addData("check ", "color");
        telemetry.update();
        sleep(debugpause2);
        colorcheck2(); // check the beacon color on the left side
        //Second colorcheck in the program

     //   sleep(debugpause);
        telemetry.addData("backward ", "4");
        telemetry.update();
      //  sleep(debugpause2);
        backward(4, 0.3, 0.0); // back up after pressing beacon


        //   robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //   robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move
        telemetry.addData("autonomous", "end");
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    ///    added function calls
    public void linefinder(double timeoutmS){
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        runtime.reset();
        // ready to find line
        while (opModeIsActive() &&  (robot.distanceSensor1.getRawLightDetected() <= 0.44) && (runtime.milliseconds() < timeoutmS)){
            telemetry.addData("dist1 ", robot.distanceSensor1.getRawLightDetected());
            telemetry.update();
            robot.leftMotor.setPower(-0.13);
            robot.rightMotor.setPower(-0.13);
        }
        telemetry.addData("dist1", robot.distanceSensor1.getRawLightDetected());
        telemetry.addData("line", " found");
        telemetry.update();
        //sleep(3000);

        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void linefollower(int timeoutmS) {
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("servo", "move");
        telemetry.update();

        double correction = 0;
        double PERFECT_COLOR_VALUE = 0.75;
        double leftPower;
        double rightPower;

        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < timeoutmS) && (robot.distanceSensor2.getRawLightDetected() < 0.30)) {
            telemetry.addData("dist2: ", robot.distanceSensor2.getRawLightDetected());

            correction = (PERFECT_COLOR_VALUE - robot.distanceSensor1.getRawLightDetected());
            correction = (correction / (PERFECT_COLOR_VALUE * 5));
            telemetry.addData("correct: ", correction);

            if (correction <= 0) {
                leftPower = 0.19 - correction;
                rightPower = 0.19 + correction;
            } else {
                leftPower = 0.19 - correction;
                rightPower = 0.19 + correction;
            }
            robot.leftMotor.setPower(-leftPower);
            robot.rightMotor.setPower(-rightPower);
            telemetry.addData("left", -leftPower);
            telemetry.addData("right", -rightPower);
            telemetry.addData("dist1", robot.distanceSensor1.getRawLightDetected());
            telemetry.update();
        }
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void colorcheck() {

        int colorred = robot.sensorRGB.red();
        int colorblue = robot.sensorRGB.blue();

        telemetry.addData("red", robot.sensorRGB.red());
        telemetry.addData("blue", robot.sensorRGB.blue());
        telemetry.update();
        sleep(200);

        if (colorblue > colorred) {
            int colordif = colorblue - colorred;
            if (colordif > 100) {
                colorresblue = true;
                colorresred = false;
                telemetry.addData("color ", "is blue");
            } else if (colordif <= 100) {
                colorresblue = false;
                colorresred = false;
                telemetry.addData("color ", "bad read!");
            }
        } else if (colorred > colorblue) {
            int colordif = colorred - colorblue;
            if (colordif > 100) {
                colorresblue = false;
                colorresred = true;
                telemetry.addData("color ", "is red");
            } else if (colordif <= 100) {
                colorresblue = false;
                colorresred = false;
                telemetry.addData("color ", "bad read!");
            }

        }
        telemetry.update();
        if (colorresblue) {
            backward(1, 0.3, 0.0);
            robot.servoL1.setPosition(0.78);
            forward(2, 0.3, 0.0); //  move forward to press button
            sleep(1000);

        } else if (colorresred){
            backward(1, 0.3, 0.0);
            robot.servoL1.setPosition(0.25);
            forward(2, 0.3, 0.0); //  move forward to press button
            sleep(1000);
        } else {
            robot.servoL1.setPosition(0.5);
        }

    }

    public void colorcheck2() {

        int colorred = robot.sensorRGB.red();
        int colorblue = robot.sensorRGB.blue();

        telemetry.addData("red", robot.sensorRGB.red());
        telemetry.addData("blue", robot.sensorRGB.blue());
        telemetry.update();
        sleep(200);

        if (colorblue > colorred) {
            int colordif = colorblue - colorred;
            if (colordif > 100) {
                colorresblue = true;
                colorresred = false;
                telemetry.addData("color ", "is blue");
            } else if (colordif <= 100) {
                colorresblue = false;
                colorresred = false;
                telemetry.addData("color ", "bad read!");
            }
        } else if (colorred > colorblue) {
            int colordif = colorred - colorblue;
            if (colordif > 100) {
                colorresblue = false;
                colorresred = true;
                telemetry.addData("color ", "is red");
            } else if (colordif <= 100) {
                colorresblue = false;
                colorresred = false;
                telemetry.addData("color ", "bad read!");
            }

        }
        telemetry.update();
        if (colorresblue) {
            backward(1, 0.3, 0.0);
            robot.servoL1.setPosition(0.78);
            forward(2, 0.3, 0.0); //  move forward to press button
            sleep(1000);

        } else if (colorresred){
            backward(1, 0.3, 0.0);
            robot.servoL1.setPosition(0.25);
            forward(2, 0.3, 0.0); //  move forward to press button
            rotate(10, 0.3, 1);
            sleep(1000);
        } else {
            robot.servoL1.setPosition(0.5);
        }

    }

    public void forward(double distance, double speed, double offset) {
        offset = offset / 2;
        double leftDist = (-distance);
        double rightDist = (-distance);
        double time = (distance * (250 * speed));
        double Lspeed = speed - offset;
        double Rspeed = speed + offset;
        encoderDrive(Lspeed, Rspeed, leftDist, rightDist, time);
    }

    public void backward(double distance, double speed, double offset) {
        offset = offset / 2;
        double leftDist = (distance);
        double rightDist = (distance);
        double time = (distance * (250 * speed));
        double Lspeed = speed - offset;
        double Rspeed = speed + offset;
        encoderDrive(Lspeed, Rspeed, leftDist, rightDist, time);
    }

    // 1 to turn left, -1 to turn right
    public void rotate(double distance, double speed, int direction) {
        double leftDist = 0;
        double angle_ref = 0.1685;
        double rightDist = 0;
        if (direction == 1) {  // 1 = left
            leftDist = (-distance * angle_ref);
            rightDist = (distance * angle_ref);
        } else if (direction == -1) { //-1 = right
            leftDist = (distance * angle_ref);
            rightDist = (-distance * angle_ref);
        }
        //double time = (distance * (10250 * speed));
        double Lspeed = speed;
        double Rspeed = speed;
        encoderDrive(Lspeed, Rspeed, leftDist, rightDist, 5.0);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double Lspeed, double Rspeed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(Lspeed));
            robot.rightMotor.setPower(Math.abs(Rspeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    public void encoderDrive2(double Lspeed, double Rspeed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(Lspeed));
            robot.rightMotor.setPower(Math.abs(Rspeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && (robot.distanceSensor1.getRawLightDetected() < 45) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}