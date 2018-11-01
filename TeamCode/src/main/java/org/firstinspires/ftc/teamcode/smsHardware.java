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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 */
public class smsHardware
{
    /* Public OpMode members. */
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  rearLeftDrive   = null;
    public DcMotor  rearRightDrive  = null;

    /* Arms */

    public DcMotor armMove = null;
    public DcMotor armExtend = null;
    public DcMotor collector = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    //private ElapsedTime period  = new ElapsedTime();


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        try
        {
            frontRightDrive = hwMap.get(DcMotor.class, "fr");
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD) ;
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            rearRightDrive = hwMap.get(DcMotor.class, "rr");
            rearRightDrive.setDirection(DcMotor.Direction.FORWARD) ;
            rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            frontLeftDrive = hwMap.get(DcMotor.class, "fl");
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE) ;
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            rearLeftDrive = hwMap.get(DcMotor.class, "rl");
            rearLeftDrive.setDirection(DcMotor.Direction.REVERSE) ;
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            armMove = hwMap.get(DcMotor.class, "am");
            armMove.setDirection(DcMotor.Direction.FORWARD);
            armMove.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMove.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception p_exception) { };

        try
        {
            armExtend = hwMap.get(DcMotor.class, "ae");
            armExtend.setDirection(DcMotor.Direction.FORWARD);
            armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExtend.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception p_exception) { };

        try
        {
            collector = hwMap.get(DcMotor.class, "c");
            collector.setDirection(DcMotor.Direction.FORWARD) ;
            collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };
    }
 }

