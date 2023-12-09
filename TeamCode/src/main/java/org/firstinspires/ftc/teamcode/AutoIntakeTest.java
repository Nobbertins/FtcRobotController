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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
This Program Allows the Tuning of The Placement of Pixels on Line for use during Autonomous

Right Trigger -> Increase Duration Of Placement
Left Trigger -> Decrease Duration Of Placement
B Button -> Increase Intake Power
Y Button -> Decrease Intake Power
Right Bumper -> Run An Placement Cycle
Left Bumper -> Reverse Intake Dircetion

 */

//define OP
@TeleOp(name="Auto Intake", group="Linear OpMode")
//@Disabled

//define OP class
public class AutoIntakeTest extends LinearOpMode {

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

    private Servo depositServo = null;

    //private CRServo contServo = null;


    //a sleep implementation in java because I read somewhere that using thread.sleep wasn't good practice with FTC OpModes
    private void timedSleep(double timeToWait){
        double startTime = getRuntime();
        while (getRuntime() - startTime < timeToWait) {
            //sleep
        }
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        extraMotor = hardwareMap.get(DcMotor.class, "rraise");
        oppMotor = hardwareMap.get(DcMotor.class, "lraise");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        lslideServo = hardwareMap.get(Servo.class, "lslide");
        rslideServo = hardwareMap.get(Servo.class, "rslide");
        ldropServo = hardwareMap.get(Servo.class, "ldrop");
        rdropServo = hardwareMap.get(Servo.class, "rdrop");
        depositServo = hardwareMap.get(Servo.class, "deposit");
        //contServo = hardwareMap.crservo.get("contservo");

        //set all default directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //initialize motor states
        boolean slideState = false;
        boolean yPressed = false;

        boolean dropState = false;
        boolean bPressed = false;

        boolean depositState = false;
        boolean aPressed = false;

        boolean intakeMotorDirection = true;
        boolean lbPressed = false;

        boolean runIntakeMotor = false;
        boolean rbPressed = false;

        //initialize servos positions
        lslideServo.setPosition(0.6);
        rslideServo.setPosition(0.1);
        depositServo.setPosition(0.5);
        //ldropServo.setPosition(1);
        rdropServo.setPosition(0.2);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);


        double intakePower = 0.75;
        //time to outake the motor period for use with autonomous
        double outakePeriod = 0.5;
        boolean intakeReverse = false;

        while (opModeIsActive()) {

            //decrease outake period gradually
            if(gamepad1.left_trigger>0) {
                outakePeriod -= 0.00001;
            }

            //increase outake period gradually
            else if(gamepad1.right_trigger>0) {
                outakePeriod += 0.00001;
            }
            //increase intake power
            bPressed = gamepad1.b;
            if (bPressed){
                intakePower += 0.00001;
            }

            //decrease intake power
            yPressed = gamepad1.y;
            if (yPressed){
                intakePower -= 0.00001;
            }

            //constrain between 0-1 range
            intakePower = Math.max(0,intakePower);
            intakePower = Math.min(1.0, intakePower);


            //reverse direction with left bumper
            lbPressed = gamepad1.left_bumper;
            if(lbPressed){
                intakeReverse = !intakeReverse;
            }


            //run with right bumper
            rbPressed = gamepad1.right_bumper;
            if (rbPressed) {
                if (intakeReverse) {
                    intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                }
                else {
                    intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                }
                //run motors for set time
                intakeMotor.setPower(intakePower);
                timedSleep(outakePeriod);
                intakeMotor.setPower(0);
                //delay between iterations for testing cycle doesn't think you are holding it down
                timedSleep(1);
            }


            else{
                intakeMotor.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake Power", "%4.2f", intakePower);
            telemetry.addData("Outake Period", "%4.2f", outakePeriod);
            telemetry.addData("Testing", "Running Correctly 1");
            telemetry.update();

        }
    }}
