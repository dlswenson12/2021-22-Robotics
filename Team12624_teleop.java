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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Team12624_teleop", group="Iterative Opmode")
//@Disabled
public class Team12624_teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
   HardwarePushbot12624 robot = new HardwarePushbot12624();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double r = Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y,-gamepad1.left_stick_x)+Math.PI/4;
        double rightx = gamepad1.right_stick_x;

        final double v1 = r*Math.cos(robotAngle) - rightx;
        final double v2 = r*Math.cos(robotAngle) + rightx;
        final double v3 = r*Math.sin(robotAngle) - rightx;
        final double v4 = r*Math.sin(robotAngle) + rightx;


        double armPower = (.5)*gamepad2.left_stick_y;
        double slidePower = gamepad2.right_stick_y;
        double servoincrement = .1;



        //double swivelPower = (.3)*gamepad2.right_stick_x;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        robot.lfDrive.setPower(v1);
        robot.lrDrive.setPower(v2);
        robot.rfDrive.setPower(v3);
        robot.rrDrive.setPower(v4);

        robot.arm.setPower(armPower);
        robot.slide.setPower(slidePower);


        if (gamepad2.a) {
            robot.wrist.setPosition(robot.wrist.getPosition()+servoincrement);

        }
        if (gamepad2.b) {
            robot.wrist.setPosition(robot.wrist.getPosition()-servoincrement);

        }

        if (gamepad2.x) {
            robot.claw.setPosition(robot.claw.getPosition()+servoincrement);

        }
        if (gamepad2.y) {
            robot.claw.setPosition(robot.claw.getPosition()-servoincrement);

        }

        if (gamepad1.a) {
            robot.hook.setPosition(robot.hook.getPosition()+servoincrement);

        }
        if (gamepad1.b) {
            robot.hook.setPosition(robot.hook.getPosition()-servoincrement);

        }


        //robot.swivel.setPower(swivelPower);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels

        // Show the elapsed game time and wheel power.

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "lfDrive (%.2f), rfDrive (%.2f), lrDrive (%.2f), rrDrive (%.2f)", v1,v2,v3,v4);
        telemetry.addData("robotAngle",robotAngle);
        telemetry.addData("rightx",rightx);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
