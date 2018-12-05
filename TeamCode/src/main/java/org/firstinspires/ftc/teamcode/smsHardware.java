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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

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

    public Servo sensorAxis = null;
    public Servo servoMarker = null;

    /* local OpMode members. */
    //HardwareMap hwMap           =  null;
    //private ElapsedTime period  = new ElapsedTime();
    public NormalizedColorSensor colorSensor = null;
    public DistanceSensor sensorRange; // used for 2m
    public DistanceSensor colorRange; // used for REV Distance+Color sensor
    public String teamID = "";

    public BNO055IMU imu = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean Auton) {
        // Save reference to Hardware map
        //hwMap = ahwMap;

        if (Auton == true) {
            // Initialize the bot-specific color sensor
            try
            {
                sensorRange = ahwMap.get(DistanceSensor.class, "2m10644");
                teamID = "10644";
            }
            catch (Exception p_exception) { };

            try
            {
                sensorRange = ahwMap.get(DistanceSensor.class, "2m10645");
                teamID = "10645";
            }
            catch (Exception p_exception) { };

            try
            {
                sensorRange = ahwMap.get(DistanceSensor.class, "2m15555");
                teamID = "15555";
            }
            catch (Exception p_exception) { };

            // Embedded IMU
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            try {
                imu = ahwMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);
            } catch (Exception p_exception) { };



            try
            {
                sensorAxis = ahwMap.get(Servo.class, "2maxis");
            }
            catch (Exception p_exception) { };

            try
            {
                servoMarker = ahwMap.get(Servo.class, "marker");
            }
            catch (Exception p_exception) { };        }

        // Define and Initialize Motors
        try
        {
            frontRightDrive = ahwMap.get(DcMotor.class, "fr");
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD) ;
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            rearRightDrive = ahwMap.get(DcMotor.class, "rr");
            rearRightDrive.setDirection(DcMotor.Direction.FORWARD) ;
            rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            frontLeftDrive = ahwMap.get(DcMotor.class, "fl");
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE) ;
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            rearLeftDrive = ahwMap.get(DcMotor.class, "rl");
            rearLeftDrive.setDirection(DcMotor.Direction.REVERSE) ;
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception) { };

        try
        {
            armMove = ahwMap.get(DcMotor.class, "am");
            armMove.setDirection(DcMotor.Direction.FORWARD);
            if (Auton == true) { armMove.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }
            armMove.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMove.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception p_exception) { };

        try
        {
            armExtend = ahwMap.get(DcMotor.class, "ae");
            armExtend.setDirection(DcMotor.Direction.FORWARD);
            if (Auton == true) { armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
            armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExtend.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception p_exception) { };

        try
        {
            collector = ahwMap.get(DcMotor.class, "c");
            collector.setDirection(DcMotor.Direction.FORWARD) ;
            collector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            collector.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception p_exception) { };
    }
 }

