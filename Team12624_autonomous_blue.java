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

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@Autonomous(name="autonomous blue", group="Pushbot")
//@Disabled
public class Team12624_autonomous_blue extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot12624 robot = new HardwarePushbot12624();  // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    private ColorSensor color2;
    private DistanceSensor sensorRange;
    int step = 0;
    int step1 = 0;
    double encoderdistance;
    double DistanceToSensor;
    double targetheading;


    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //setup color sensor
        color2 = hardwareMap.get(ColorSensor.class, "color_sensor");


        // setup distance sensor
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensorRange");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        View relativeLayout;
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        // Send telemetry message to signify robot waiting;
        boolean lastResetState = false;
        boolean curResetState = false;

        // setup gyroscope

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        // A similar approach will work for the Gyroscope interface, if that's all you need.

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();


        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.lfDrive.getCurrentPosition(),
                robot.rfDrive.getCurrentPosition(),
                robot.lrDrive.getCurrentPosition(),
                robot.rrDrive.getCurrentPosition());
        telemetry.update();
        //setup float variables for color sensor
        //hsvValues is an array that willl hold the hue, saturation and value info
        float hsvValues[] = {0F, 0F, 0F};
        //values is a reference to the hsvValues array
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        // Wait for the game to start (driver presses PLAY)


        waitForStart();


        telemetry.addData("Step:  ", step);

        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();

        // Step through each leg of the path,=
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        telemetry.addData("Step1: hook up  ", step+1);
        robot.hook.setPosition(0);
        sleep(500);

        telemetry.addData("Step2: drive forward to next mat ", step+2);
        gyroDrive(DRIVE_SPEED, 0, -11, -11, 11, 11, 5);
        sleep(500);

        telemetry.addData("Step3: turn right 90 ", step+3);
        gyroDrive(TURN_SPEED, 90, 10, 10, 10, 10, 3);
        sleep(500);

        telemetry.addData("Step4: forward to wall ", step+4);
        gyrodistancesensorDrive(DRIVE_SPEED,0,-77,-77,77,77,15,10);
        sleep(500);

        telemetry.addData("Step5: turn left 90 ", step+5);
        gyroDrive(TURN_SPEED, -90, -11, -11, -11, -11, 3);
        sleep(500);

        telemetry.addData("Step6: drive forward to foundation ", step+6);
        gyrocolorsensorDrive(.15,0,-9,-9,9,9,215,2);
        sleep(500);

        telemetry.addData("Step7: hook down  ", step+7);
        robot.hook.setPosition(0);
        sleep(500);

        telemetry.addData("Step8: backup to wall ", step+8);
        gyroDrive(TURN_SPEED, 0, 12, 12, -12, -12, 3);
        sleep(500);

        telemetry.addData("Step9: hook up  ", step+9);
        robot.hook.setPosition(1);
        sleep(500);


/*
        telemetry.addData("Step10: turn left 90 ", step+10);
        gyroDrive(TURN_SPEED, -90, 11, 11, 11, 11, 3);
        sleep(500);

        telemetry.addData("Step11: strafe left to line ", step+11);
        gyrocolorsensorDrive(DRIVE_SPEED,0,-80,80,80,-80,0,3);
        sleep(500);

*/
        //encoderDrive(TURN_SPEED,   11, 11, 11, 11, 8.0);  //turn left 11 inches
        sleep(1000);


        //drive forward until color sensor is activated
        while (opModeIsActive()) {
            Color.RGBToHSV((int) (color2.red() * SCALE_FACTOR),
                    (int) (color2.green() * SCALE_FACTOR),
                    (int) (color2.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Step:  ", step);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

/*
        while (step == 0 && opModeIsActive()){

                Color.RGBToHSV((int)(color2.red()*SCALE_FACTOR),
                        (int)(color2.green()*SCALE_FACTOR),
                        (int) (color2.blue()*SCALE_FACTOR),
                        hsvValues);

             robot.lfDrive.setPower(-.25);
             robot.lrDrive.setPower(-.25);
             robot.rfDrive.setPower(.25);
             robot.rrDrive.setPower(.25);

            if (hsvValues [0] < 100) {
                step++;
                robot.lfDrive.setPower(0);
                robot.lrDrive.setPower(0);
                robot.rfDrive.setPower(0);
                robot.rrDrive.setPower(0);

            }

          };

 */
            //robot.hook.setPosition(0);
            sleep(1000);
            // encoderDrive(DRIVE_SPEED,  -30,  -30,30, 30, 2.0);  //backup 30 inches to wall

        }
        ;

        sleep(5000);


        // S4: Stop and close the claw.
        //robot.wrist.setPosition(0.0);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void gyroDrive(double speed, double targetheading, double lfInches, double lrInches, double rfInches, double rrInches,
                          double timeoutS) {
        int newlfTarget;
        int newlrTarget;
        int newrfTarget;
        int newrrTarget;
        double speedchange = .2;
        double actualheading;
        double headingerror;

        modernRoboticsI2cGyro.resetZAxisIntegrator();


        // Ensure that the opmode is still active
        if (opModeIsActive()) {




            //reset encoders
            robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newlfTarget = robot.lfDrive.getCurrentPosition() + (int) (lfInches * COUNTS_PER_INCH);
            newlrTarget = robot.lrDrive.getCurrentPosition() + (int) (lrInches * COUNTS_PER_INCH);
            newrfTarget = robot.rfDrive.getCurrentPosition() + (int) (rfInches * COUNTS_PER_INCH);
            newrrTarget = robot.rrDrive.getCurrentPosition() + (int) (rrInches * COUNTS_PER_INCH);

            robot.lfDrive.setTargetPosition(newlfTarget);
            robot.lrDrive.setTargetPosition(newlrTarget);
            robot.rfDrive.setTargetPosition(newrfTarget);
            robot.rrDrive.setTargetPosition(newrrTarget);



            // Turn On RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&  (robot.lfDrive.isBusy() || robot.lrDrive.isBusy() || robot.rfDrive.isBusy() || robot.rrDrive.isBusy())) {
                int heading = modernRoboticsI2cGyro.getHeading();
                int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
                telemetry.addData("heading", "%3d deg", heading);
                telemetry.addData("integrated Z", "%3d", integratedZ);

                actualheading = integratedZ;
                headingerror = actualheading - targetheading;

                telemetry.update();

                telemetry.addData("headingerror", headingerror);


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newlfTarget, newlrTarget, newrfTarget, newrrTarget);
                telemetry.addData("actual", "Running to %7d :%7d :%7d :%7d", robot.lfDrive.getCurrentPosition(), robot.lrDrive.getCurrentPosition(), robot.rfDrive.getCurrentPosition(), robot.rrDrive.getCurrentPosition());
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.lfDrive.getCurrentPosition(),
                        robot.lrDrive.getCurrentPosition(),
                        robot.rfDrive.getCurrentPosition(),
                        robot.rrDrive.getCurrentPosition());
                if (headingerror == 0) {
                    robot.lfDrive.setPower(Math.abs(speed));
                    robot.lrDrive.setPower(Math.abs(speed));
                    robot.rfDrive.setPower(Math.abs(speed));
                    robot.rrDrive.setPower(Math.abs(speed));
                }
                if (headingerror < -1) {
                    robot.lfDrive.setPower(Math.abs(speed ));
                    robot.lrDrive.setPower(Math.abs(speed ));
                    robot.rfDrive.setPower(Math.abs(speed + speedchange));
                    robot.rrDrive.setPower(Math.abs(speed + speedchange));
                }
                if (headingerror > 1) {
                    robot.lfDrive.setPower(Math.abs(speed + speedchange));
                    robot.lrDrive.setPower(Math.abs(speed + speedchange));
                    robot.rfDrive.setPower(Math.abs(speed ));
                    robot.rrDrive.setPower(Math.abs(speed ));
                }
                telemetry.update();
            }

            // Stop all motion;
            robot.lfDrive.setPower(0);
            robot.lrDrive.setPower(0);
            robot.rfDrive.setPower(0);
            robot.rrDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }


    }



    public void gyrodistancesensorDrive(double speed, double targetheading, double lfInches, double lrInches, double rfInches, double rrInches,double minDistance,
                                        double timeoutS) {
        int newlfTarget;
        int newlrTarget;
        int newrfTarget;
        int newrrTarget;
        double speedchange = .2;
        double actualheading;
        double headingerror;

        modernRoboticsI2cGyro.resetZAxisIntegrator();


        // Ensure that the opmode is still active
        if (opModeIsActive()) {




            //reset encoders
            robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Determine new target position, and pass to motor controller
            newlfTarget = robot.lfDrive.getCurrentPosition() + (int) (lfInches * COUNTS_PER_INCH);
            newlrTarget = robot.lrDrive.getCurrentPosition() + (int) (lrInches * COUNTS_PER_INCH);
            newrfTarget = robot.rfDrive.getCurrentPosition() + (int) (rfInches * COUNTS_PER_INCH);
            newrrTarget = robot.rrDrive.getCurrentPosition() + (int) (rrInches * COUNTS_PER_INCH);

            robot.lfDrive.setTargetPosition(newlfTarget);
            robot.lrDrive.setTargetPosition(newlrTarget);
            robot.rfDrive.setTargetPosition(newrfTarget);
            robot.rrDrive.setTargetPosition(newrrTarget);



            // Turn On RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            DistanceToSensor = sensorRange.getDistance(DistanceUnit.INCH);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&  (robot.lfDrive.isBusy() || robot.lrDrive.isBusy() || robot.rfDrive.isBusy() || robot.rrDrive.isBusy()) && (DistanceToSensor > minDistance)) {
                int heading = modernRoboticsI2cGyro.getHeading();
                int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
                telemetry.addData("heading", "%3d deg", heading);
                telemetry.addData("integrated Z", "%3d", integratedZ);

                telemetry.addData("deviceName", sensorRange.getDeviceName());
                telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));


                actualheading = integratedZ;
                headingerror = actualheading - targetheading;
                DistanceToSensor = sensorRange.getDistance(DistanceUnit.INCH);
                telemetry.update();

                telemetry.addData("headingerror", headingerror);


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newlfTarget, newlrTarget, newrfTarget, newrrTarget);
                telemetry.addData("actual", "Running to %7d :%7d :%7d :%7d", robot.lfDrive.getCurrentPosition(), robot.lrDrive.getCurrentPosition(), robot.rfDrive.getCurrentPosition(), robot.rrDrive.getCurrentPosition());
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.lfDrive.getCurrentPosition(),
                        robot.lrDrive.getCurrentPosition(),
                        robot.rfDrive.getCurrentPosition(),
                        robot.rrDrive.getCurrentPosition());
                if (headingerror == 0) {
                    robot.lfDrive.setPower(Math.abs(speed));
                    robot.lrDrive.setPower(Math.abs(speed));
                    robot.rfDrive.setPower(Math.abs(speed));
                    robot.rrDrive.setPower(Math.abs(speed));
                }
                if (headingerror < -1) {
                    robot.lfDrive.setPower(Math.abs(speed ));
                    robot.lrDrive.setPower(Math.abs(speed ));
                    robot.rfDrive.setPower(Math.abs(speed + speedchange));
                    robot.rrDrive.setPower(Math.abs(speed + speedchange));
                }
                if (headingerror > 1) {
                    robot.lfDrive.setPower(Math.abs(speed + speedchange));
                    robot.lrDrive.setPower(Math.abs(speed + speedchange));
                    robot.rfDrive.setPower(Math.abs(speed ));
                    robot.rrDrive.setPower(Math.abs(speed ));
                }
                telemetry.update();
            }

            // Stop all motion;
            robot.lfDrive.setPower(0);
            robot.lrDrive.setPower(0);
            robot.rfDrive.setPower(0);
            robot.rrDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }


    }


    public void gyrocolorsensorDrive(double speed, double targetheading, double lfInches, double lrInches, double rfInches, double rrInches,double huevalue,
                                     double timeoutS) {
        int newlfTarget;
        int newlrTarget;
        int newrfTarget;
        int newrrTarget;
        double speedchange = .2;
        double actualheading;
        double headingerror;

        modernRoboticsI2cGyro.resetZAxisIntegrator();


        // Ensure that the opmode is still active
        if (opModeIsActive()) {




            //reset encoders
            robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Determine new target position, and pass to motor controller
            newlfTarget = robot.lfDrive.getCurrentPosition() + (int) (lfInches * COUNTS_PER_INCH);
            newlrTarget = robot.lrDrive.getCurrentPosition() + (int) (lrInches * COUNTS_PER_INCH);
            newrfTarget = robot.rfDrive.getCurrentPosition() + (int) (rfInches * COUNTS_PER_INCH);
            newrrTarget = robot.rrDrive.getCurrentPosition() + (int) (rrInches * COUNTS_PER_INCH);

            robot.lfDrive.setTargetPosition(newlfTarget);
            robot.lrDrive.setTargetPosition(newlrTarget);
            robot.rfDrive.setTargetPosition(newrfTarget);
            robot.rrDrive.setTargetPosition(newrrTarget);



            // Turn On RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&  (robot.lfDrive.isBusy() || robot.lrDrive.isBusy() || robot.rfDrive.isBusy() || robot.rrDrive.isBusy()) && (huevalue > huevalue-15 && huevalue<huevalue+15)) {
                int heading = modernRoboticsI2cGyro.getHeading();
                int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
                telemetry.addData("heading", "%3d deg", heading);
                telemetry.addData("integrated Z", "%3d", integratedZ);

                actualheading = integratedZ;
                headingerror = actualheading - targetheading;

                telemetry.update();

                telemetry.addData("headingerror", headingerror);


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newlfTarget, newlrTarget, newrfTarget, newrrTarget);
                telemetry.addData("actual", "Running to %7d :%7d :%7d :%7d", robot.lfDrive.getCurrentPosition(), robot.lrDrive.getCurrentPosition(), robot.rfDrive.getCurrentPosition(), robot.rrDrive.getCurrentPosition());
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.lfDrive.getCurrentPosition(),
                        robot.lrDrive.getCurrentPosition(),
                        robot.rfDrive.getCurrentPosition(),
                        robot.rrDrive.getCurrentPosition());
                if (headingerror == 0) {
                    robot.lfDrive.setPower(Math.abs(speed));
                    robot.lrDrive.setPower(Math.abs(speed));
                    robot.rfDrive.setPower(Math.abs(speed));
                    robot.rrDrive.setPower(Math.abs(speed));
                }
                if (headingerror < -1) {
                    robot.lfDrive.setPower(Math.abs(speed ));
                    robot.lrDrive.setPower(Math.abs(speed ));
                    robot.rfDrive.setPower(Math.abs(speed + speedchange));
                    robot.rrDrive.setPower(Math.abs(speed + speedchange));
                }
                if (headingerror > 1) {
                    robot.lfDrive.setPower(Math.abs(speed + speedchange));
                    robot.lrDrive.setPower(Math.abs(speed + speedchange));
                    robot.rfDrive.setPower(Math.abs(speed ));
                    robot.rrDrive.setPower(Math.abs(speed ));
                }
                telemetry.update();
            }

            // Stop all motion;
            robot.lfDrive.setPower(0);
            robot.lrDrive.setPower(0);
            robot.rfDrive.setPower(0);
            robot.rrDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }


    }

/*
    public void encoderDrive(double speed,
                             double lfInches, double lrInches, double rfInches, double rrInches,
                             double timeoutS) {
        int newlfTarget;
        int newlrTarget;
        int newrfTarget;
        int newrrTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newlfTarget = robot.lfDrive.getCurrentPosition() + (int)(lfInches * COUNTS_PER_INCH);
            newlrTarget = robot.lrDrive.getCurrentPosition() + (int)(lrInches * COUNTS_PER_INCH);
            newrfTarget = robot.rfDrive.getCurrentPosition() + (int)(rfInches * COUNTS_PER_INCH);
            newrrTarget = robot.rrDrive.getCurrentPosition() + (int)(rrInches * COUNTS_PER_INCH);

            robot.lfDrive.setTargetPosition(newlfTarget);
            robot.lrDrive.setTargetPosition(newlrTarget);
            robot.rfDrive.setTargetPosition(newrfTarget);
            robot.rrDrive.setTargetPosition(newrrTarget);

            // Turn On RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lfDrive.setPower(Math.abs(speed));
            robot.lrDrive.setPower(Math.abs(speed));
            robot.rfDrive.setPower(Math.abs(speed));
            robot.rrDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.lfDrive.isBusy() && robot.lrDrive.isBusy() && robot.rfDrive.isBusy() && robot.rrDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newlfTarget,  newlrTarget, newrfTarget, newrrTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            robot.lfDrive.getCurrentPosition(),
                                            robot.lrDrive.getCurrentPosition(),
                                            robot.rfDrive.getCurrentPosition(),
                                            robot.rrDrive.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.lfDrive.setPower(0);
            robot.lrDrive.setPower(0);
            robot.rfDrive.setPower(0);
            robot.rrDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
    */


}
