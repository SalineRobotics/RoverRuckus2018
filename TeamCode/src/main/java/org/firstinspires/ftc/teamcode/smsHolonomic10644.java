package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.smsHardware;

@TeleOp(name="10644", group="Pushbot")
public class smsHolonomic10644 extends LinearOpMode {

    /* Declare OpMode members. */
    smsHardware robot = new smsHardware();   // Use a Pushbot's hardware
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;
    float armNominalPower = 0.3f;
    float driveNominalPower = 0.3f;
    int amPos;
    int aePos;
    int aeOffset;
    boolean previousDPD = false;
    boolean previousDPU = false;
    boolean previousDPL = false;
    boolean previousDPR = false;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        float powerReducer = 0.5f;
        // Wait for the game to start (driver presses PLAY)


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            // holonomic formulas
            /*float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            */

            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontLeft = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;

            float gamepad2LeftY = -gamepad2.left_stick_y;
            float gamepad2RightY = -gamepad2.right_stick_y;
            float gamepad2RightTrigger = gamepad2.right_trigger;
            float gamepad2LeftTrigger = gamepad2.left_trigger;

            // Allow driver to select Tank vs POV by pressing START
            boolean dpad_check = gamepad2.dpad_up;
            if (dpad_check && (dpad_check != previousDPU)) {
                aeOffset += 25;
            }
            previousDPU = dpad_check;

            dpad_check = gamepad2.dpad_down;
            if (dpad_check && (dpad_check != previousDPD)) {
                aeOffset -= 25;
            }
            previousDPD = dpad_check;

            dpad_check = gamepad2.dpad_left;
            if (dpad_check && (dpad_check != previousDPL)) {
                armNominalPower -= 0.05;
            }
            previousDPL = dpad_check;

            dpad_check = gamepad2.dpad_right;
            if (dpad_check && (dpad_check != previousDPR)) {
                armNominalPower += 0.05;
            }
            previousDPR = dpad_check;

            float armMove = Range.clip(gamepad2LeftY, -1, 1) * armNominalPower;  // cap the arm-move to 50% but without clipping
            float armEx = Range.clip(gamepad2RightY, -1, 1) * 0.2f;   // cap the arm-extend to 20% but without clipping

            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            powerReducer = driveNominalPower;
            if (gamepad1.right_trigger > 0) {
                powerReducer = 1.0f;
            }
            if (gamepad1.left_trigger > 0) {
                powerReducer = 0.1f;
            }

            // write the values to the motors

            if (robot.frontRightDrive != null) {
                robot.frontRightDrive.setPower(FrontRight * powerReducer);
            }
            if (robot.frontLeftDrive != null) {
                robot.frontLeftDrive.setPower(FrontLeft * powerReducer);
            }
            if (robot.rearLeftDrive != null) {
                robot.rearLeftDrive.setPower(BackLeft * powerReducer);
            }
            if (robot.rearRightDrive != null) {
                robot.rearRightDrive.setPower(BackRight * powerReducer);
            }

            if (robot.collector != null) {
                if (gamepad2LeftTrigger > 0f) {
                    robot.collector.setPower(1);
                } else if (gamepad2RightTrigger > 0f) {
                    robot.collector.setPower(-1);
                } else {
                    robot.collector.setPower(0);
                }
            }

            if (robot.armMove != null) {
                robot.armMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (gamepad2.y) {
                    amPos = 2500;
                    armMove = armNominalPower;
                } else if (gamepad2.a) {
                    amPos = 0;
                    armMove = 1.0f;
                } else {
                    amPos = (int) (robot.armMove.getCurrentPosition() + armMove / Math.abs(armMove) * 100);
                    amPos = Range.clip(amPos, 0, 6000);
                }
                robot.armMove.setTargetPosition(amPos);
                robot.armMove.setPower(Math.abs(armMove));

                amPos = robot.armMove.getCurrentPosition();
            }

            if (robot.armExtend != null) {
                //this code is for the motorized collector
                robot.armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armExtend.setTargetPosition((int) (aeOffset + amPos / 7.11)); // based on a double 15:40 tooth reduction setup
                robot.armExtend.setPower(0.2f);

                // this code is for the 4-link / passive collector
                //robot.armExtend.setPower(armEx);
                //aePos = robot.armExtend.getCurrentPosition();
            }

            //print out motor values
            telemetry.addLine()
                    .addData("front right", FrontRight)
                    .addData("front left", FrontLeft)
                    .addData("back left", BackLeft)
                    .addData("back right", BackRight)
                    .addData("armExtend ", aePos)
                    .addData("armExtendPower", armEx)
                    .addData("armMove ", amPos)
                    .addData("armMovePower", armMove);

            telemetry.update();
        }
    }
}