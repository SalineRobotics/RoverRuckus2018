package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="LAND", group="Pushbot")
public class smsLAND extends LinearOpMode {

    /* Declare OpMode members. */
    smsHardware robot = new smsHardware();   // Use a Pushbot's hardware

    float[] hsvValues = new float[3];
    final float values[] = hsvValues;
    int amPos;
    int aePos;
    int aeOffset;
    boolean previousDPD = false;
    boolean previousDPU = false;
    boolean previousDPL = false;
    boolean previousDPR = false;
    float armNominalPower = 0.3f;
    float driveNominalPower = 0.3f;
    
    @Override
    public void runOpMode() {
        float powerReducer = driveNominalPower;

        robot.init(hardwareMap, false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        aeOffset = 0;
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           float gamepad2LeftY = gamepad2.left_stick_y;
           float gamepad2RightY = -gamepad2.right_stick_y;

             // Allow driver to select Tank vs POV by pressing START
            boolean dpad_check = gamepad2.dpad_up;
            if(dpad_check && (dpad_check != previousDPU)) {
                aeOffset += 25;
            }
            previousDPU = dpad_check;

            dpad_check = gamepad2.dpad_down;
            if(dpad_check && (dpad_check != previousDPD)) {
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

            float armMove = Range.clip(gamepad2LeftY,-1,1)*armNominalPower;  // cap the arm-move to 50% but without clipping
            float armEx = Range.clip(gamepad2RightY,-1,1)*0.2f;   // cap the arm-extend to 20% but without clipping

            if (robot.armMove != null) {
                robot.armMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (gamepad2.y) {
                    amPos = 2500;
                    armMove = armNominalPower;
                } else {
                    amPos = (int) (robot.armMove.getCurrentPosition() + armMove / Math.abs(armMove) * 100);
                    amPos = Range.clip(amPos,-1000,5500);
                }

                if (gamepad2.a) {
                    amPos = 0;
                    armMove = (int) 1.0;
                }

                robot.armMove.setTargetPosition(amPos);
                robot.armMove.setPower(Math.abs(armMove));

//                    robot.armMove.setTargetPosition(robot.armMove.getCurrentPosition)

//                if (gamepad2.y) {
                    //robot.armMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.armMove.setTargetPosition(2500);
//                    robot.armMove.setPower(armNominalPower);
//                } else {
//                    if (robot.armMove.isBusy()) {

//                    } else {

//                        robot.armMove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        robot.armMove.setPower(armMove);
//                    }
//                }
                amPos = robot.armMove.getCurrentPosition();
            }




            if (robot.armExtend != null) {
                robot.armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Keep aligned with main arm Position (amPos)
                aePos = (int) (robot.armExtend.getCurrentPosition() + armEx / Math.abs(armEx) * 100); //(int)(aeOffset + Range.clip(amPos,0,5500) / 7.11);
                // Dump
                //if (gamepad2.b){ aePos += 1000; }
                robot.armExtend.setTargetPosition(aePos); // based on a double 15:40 tooth reduction setup
                robot.armExtend.setPower(0.2f);
//                robot.armExtend.setPower(armEx);
                aePos = robot.armExtend.getCurrentPosition();

            }



            telemetry.addLine()
                    .addData("armExtend ", aePos)
                    .addData("armExtendPower",armEx)
                    .addData("armMove ", amPos)
                    .addData("armMovePower",armMove);

            telemetry.update();

        }
    }
}