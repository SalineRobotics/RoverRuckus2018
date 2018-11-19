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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.smsHardware;

import java.util.Locale;


@TeleOp(name="Pushbot: Color Sensor", group="Pushbot")
public class ColorSensorTest extends LinearOpMode {

    /* Declare OpMode members. */
    NormalizedColorSensor colorSensor;
    DistanceSensor sensorRange;
    DistanceSensor sensorDistance;
    Servo sensorAxis;
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;
boolean previousDPU;
boolean previousDPD;

    @Override
    public void runOpMode() {
        boolean bPrevState = false;
        boolean bCurrState = false;
        double S = 0.0;
        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "2m");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cs15555");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "cs15555");
        sensorAxis = hardwareMap.get(Servo.class,"2maxis");
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */





        // Wait for the game to start (driver presses PLAY)

        if (colorSensor instanceof SwitchableLight) {
            telemetry.addData("color sensor", "Is Switchable");
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(false);
        } else {
            telemetry.addData("color sensor", "Is NOT Switchable");
        }


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.y) { S = 1.0; }
            if (gamepad2.x) { S = 0.25; }
            if (gamepad2.b) { S = 0.75; }
            if (gamepad2.a) { S= 1.0; }
            boolean dpad_check = gamepad2.dpad_up;
            if(dpad_check && (dpad_check != previousDPU)) {
                S += 0.1;
            }
            previousDPU = dpad_check;

            dpad_check = gamepad2.dpad_down;
            if(dpad_check && (dpad_check != previousDPD)) {
                S -= 0.1;
            }
            previousDPD = dpad_check;

            S = Range.clip(S,0.0,1.0);
            sensorAxis.setPosition(S);

            //COLOR SENSOR
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);

            int color = colors.toColor();
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red   /= max;
            colors.green /= max;
            colors.blue  /= max;
            color = colors.toColor();
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Distance (cm)",
                    sensorDistance.getDistance(DistanceUnit.CM));

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color))
                    .addData("r1",  (float)(Color.red(color)*100/Color.blue(color)*100))
                    .addData("r2",  (float)(Color.red(color)*100/Color.blue(color)));
            if (hsvValues[1] < .5){
                telemetry.addLine()
                        .addData("Color", "White");
            }
            else{
                telemetry.addLine()
                        .addData("Color", "Yellow");
            }



                    // generic DistanceSensor methods.
                    telemetry.addData("deviceName",sensorRange.getDeviceName() );
                    telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
                    telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
                    telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
                    telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

                    // Rev2mDistanceSensor specific methods.
                    //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                    //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
            /*

            bCurrState = gamepad1.x;

            if (bCurrState != bPrevState) {
                if (bCurrState) {
                    if (colorSensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight)colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }

            bPrevState = bCurrState;
            */
            telemetry.update();


            sleep(50);
        }
    }
}