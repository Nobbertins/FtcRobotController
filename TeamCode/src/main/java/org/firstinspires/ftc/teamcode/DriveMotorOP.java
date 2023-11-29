/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

/*
CONTROLLER LAYOUT:
Two joysticks - driving
A - run intake motor
LT - linear slide up
RT - linear slide down
Y - toggle slide servos
B - toggle drop servos

CONFIG MOTOR NAMES:
Control Hub:
Motors:
0 - fl
1 - fr
2 - br
3 - bl
Servos:
0 - lslide
1 - rslide
2 - ldrop
3 - rdrop
4 - deposit
Expansion Hub:
Motors:
0 - extramotor
1 - oppmotor
2 - intake
 */

//define OP
@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled

//define OP class
public class DriveMotorOP extends LinearOpMode {

    // declare time counter variable
    private ElapsedTime runtime = new ElapsedTime();

    //declare al DCMotors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor extraMotor = null;

    private DcMotor oppMotor = null;

    private DcMotor intakeMotor = null;
    //declare all servo motors
    private Servo lslideServo = null;

    private Servo rslideServo = null;

    private Servo ldropServo = null;

    private Servo rdropServo = null;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        extraMotor = hardwareMap.get(DcMotor.class, "extramotor");
        oppMotor = hardwareMap.get(DcMotor.class, "oppmotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        lslideServo = hardwareMap.get(Servo.class, "lslide");
        rslideServo = hardwareMap.get(Servo.class, "rslide");
        ldropServo = hardwareMap.get(Servo.class, "ldrop");
        rdropServo = hardwareMap.get(Servo.class, "rdrop");


        //set all default directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //initialize servo states
        boolean slideState = false;
        boolean yPressed = false;

        boolean dropState = false;
        boolean bPressed = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //max variable will be used later -> calculated then compared
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            //compare max to 1.0 which is 100%
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //switch slide servos position on y press
            if(gamepad1.y) {
                if (!yPressed) {
                    slideState = !slideState;
                    if (slideState) {
                        lslideServo.setPosition(0);
                        rslideServo.setPosition(90);
                    } else {
                        lslideServo.setPosition(90);
                        rslideServo.setPosition(0);
                    }
                }
            }
            yPressed = gamepad1.y;
            //switch drop servos position on b press
            if(gamepad1.b) {
                if (!bPressed) {
                    dropState = !dropState;
                    if (dropState) {
                        ldropServo.setPosition(0);
                        rdropServo.setPosition(90);
                    } else {
                        ldropServo.setPosition(90);
                        rdropServo.setPosition(0);
                    }
                }
            }
            bPressed = gamepad1.b;
            //both triggers are activated then do nothing to prevent killing the motors
            if(gamepad1.left_trigger>0 && gamepad1.right_trigger>0){
                extraMotor.setPower(0);
                oppMotor.setPower(0);
            }
            //left trigger linear slide go up
            else if(gamepad1.left_trigger>0) {
                extraMotor.setDirection(DcMotor.Direction.FORWARD);
                oppMotor.setDirection(DcMotor.Direction.REVERSE);
                oppMotor.setPower(0.5);
                extraMotor.setPower(0.5);
            }
            //right trigger linear slide go down
            else if(gamepad1.right_trigger>0){
                extraMotor.setDirection(DcMotor.Direction.REVERSE);
                oppMotor.setDirection(DcMotor.Direction.FORWARD);
                oppMotor.setPower(0.75);
                extraMotor.setPower(0.75);
                //up
            }
            //otherwise do nothing
            else{
                extraMotor.setPower(0);
                oppMotor.setPower(0);
            }
            //while holding "A" button, run intake motor at full power
            if(gamepad1.a){
                intakeMotor.setPower(1);
            }
            //when released stop intake motor
            else{
                intakeMotor.setPower(0);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Slide left/Right Position", "%4.2f, %4.2f", lslideServo.getPosition(), rslideServo.getPosition());
            telemetry.addData("Servo Drop left/Right Position", "%4.2f, %4.2f", ldropServo.getPosition(), rdropServo.getPosition());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}
