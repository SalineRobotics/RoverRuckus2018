package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


public class smsAuton extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXKT92L/////AAAAGS+ZuWWofUShhb1MB4+Zbic/fnrONEJsEKNCY4RE1F7X8GaFg4EQYqHF4GMlj35ZJdzZ/LQlnXVV2WlhqhHR5IDlScqWtishwl2yPBRzCXAWYP5MCphLOigzPcshkggMYEKQWxwlhvoc2lsN+54KexfxlI0ss9cMq+unSD8ZZ5Of5OuY0lX7DWAEEPh1KsdeEU7EkCGP96f5TQI518LsriyHeg73KgDLCcGd0yBUSuGWTTV3o/cTRziN+Ac1sYNzw1sEddiBS2TfCdjRlY2qMmgyAMARQhYEbcqbzGz8jcDNOsX/gS/knjAZ9UYPZl7mYFyq3Acg3089CTN+EXkFEMJysFU0XQW9P2YzICsAivi5";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    smsHardware robot = new smsHardware();   // Use a Pushbot's hardware
    smsJSON settings = new smsJSON();
    ElapsedTime runtime = new ElapsedTime();

    static final double HEADING_THRESHOLD = 3;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.15;     // Larger is more responsive, but also less stable
    //DcMotor leftMotor = null;
//DcMotor rightMotor = null;
//BNO055IMU imu = null;                    // Additional Gyro device
    int CryptoBoxOffset = 0;
//VuforiaLocalizer RobotVision; // The Vuforia application.
//VuforiaTrackables relicTrackables; // The Relic image resource file.
//VuforiaTrackable relicTemplate; // The image referenced in the resource file.

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, true);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

// Initialize the variables needed to store are JSON auton parameters
        int ArraySize = 50;
        double[] leftArray;
        leftArray = new double[ArraySize];
        double[] rightArray;
        rightArray = new double[ArraySize];
        double[] timeArray;
        timeArray = new double[ArraySize];
        int[] v_state;
        v_state = new int[ArraySize];
        int v_state_current = 0;
        double servoPOS = 0.25;
        double servoPOS1 = 0.75;
        double servoPOS2 = 0.25;
/*

double r = 0;
double b = 0;
double n = 0;

int mdelta = 0;
int dpad = 1;
int calibration = 60;
int forkdelay = 80;

// These values are OpMode specific
// RED Alliance:  -1/1
// BLUE Alliance: 1/-1
//int REDalliance = 1;
//int BLUEalliance;

// tri_state 0 - Programming
// tri_state 1 - Autonomous
// tri_state 2 - Save
int tri_state_default = 1;
if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) { tri_state_default = 2; }


// 0 - do nothing
// 1 - timed drive
// 2 - drive
// 3 - detect white line
// 4 - push beacon (capture or steal depending on tri_state)
// 5 - shoot
// 6 - open jewel
*/
// Shared Code Below
        OpModeManagerImpl opModeManager = (OpModeManagerImpl) this.internalOpModeServices; //Store OpModeManagerImpl
        String OpModeName = robot.teamID + opModeManager.getActiveOpModeName() + ".json";
        if (robot.sensorAxis != null) robot.sensorAxis.setPosition(servoPOS);
        if (robot.servoMarker != null) robot.servoMarker.setPosition(1.0);
        settings.ReadSettings(OpModeName);
        v_state_current = 0;
        // Read the first step of the state machine
        v_state[v_state_current] = settings.GetIntSetting(String.format("v_state%02d", v_state_current));
        // Keep Reading until all steps are read, don't read anything we don't actually need
        while (v_state[v_state_current] > 0) {
            timeArray[v_state_current] = settings.GetSetting(String.format("timeArray%02d", v_state_current));
            rightArray[v_state_current] = settings.GetSetting(String.format("rightArray%02d", v_state_current));
            leftArray[v_state_current] = settings.GetSetting(String.format("leftArray%02d", v_state_current));
            v_state_current++;
            v_state[v_state_current] = settings.GetIntSetting(String.format("v_state%02d", v_state_current));
        }
        telemetry.addData("name",OpModeName);
        telemetry.addData("Status", v_state_current);
        telemetry.update();
        v_state_current = 0;

// last line - end of state machine

        double intRunTime = 0;
/*
float hsvValues[] = {0F, 0F, 0F};
float hsv;

// values is a reference to the hsvValues array.
final float values[] = hsvValues;
DcMotor dumpMotor = null;
DcMotor liftMotor = null;
DcMotor slideMotor = null;
DcMotor armMotor = null;
ColorSensor sensorRGB = null;
Servo clawServo = null;
Servo rightServo = null;
Servo leftServo = null;
Servo jewelServo = null;

double left = 0.0;
double right = 0.0;
double up = 0.0;
double down = 0.0;
double collect = 0.0;
double shoot = 0.0;

double clawPosition = 0.0;
double handPosition = 0.5;
double handOffset = 0.0;
double beaconPosition = 0.5;
double jewelPositionUp = 0.95;
double jewelPositionDn = 0.36;


int tri_state;

int inverse = 1;
double multiplier1 = 1;
double multiplier2 = 1;

boolean bPrevStateA = false;
boolean bCurrStateA = false;
boolean bPrevStateB = false;
boolean bCurrStateB = false;
boolean bPrevStateX = false;
boolean bCurrStateX = false;
boolean bPrevStateY = false;
boolean bCurrStateY = false;

boolean bPrevGuide = false;
boolean bCurrGuide = false;

boolean bPrevdpadUP = false;
boolean bCurrdpadUP = false;
boolean bPrevdpadDOWN = false;
boolean bCurrdpadDOWN = false;
boolean bPrevdpadLEFT = false;
boolean bCurrdpadLEFT = false;
boolean bPrevdpadRIGHT = false;
boolean bCurrdpadRIGHT = false;

BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
parameters.loggingEnabled = true;
parameters.loggingTag = "IMU";
//parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
try {
imu = hardwareMap.get(BNO055IMU.class, "imu");
imu.initialize(parameters);
} catch (Exception p_exception) {
imu = null;
}
double max = 0.0;
Servo dummy = null;


//if (clawServo != null) clawServo.setPosition(clawPosition);
//if (rightServo != null) rightServo.setPosition(handPosition-handOffset);
//if (leftServo != null) leftServo.setPosition(handPosition+handOffset);
//if (jewelServo != null) jewelServo.setPosition(jewelPositionUp);

*/


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


/*
tri_state = tri_state_default;

bCurrStateX = gamepad1.x;
// check for button-press state transitions.
if ((bCurrStateX) && (bCurrStateX != bPrevStateX)) {
v_state_current = 0;                // button is transitioning to a pressed state.  Reset the v_state
}
// update previous state variable.
bPrevStateX = bCurrStateX;
if (bCurrStateX) {
tri_state = 2;
}

// Determine if we are currently stealing a beacon
bCurrStateB = gamepad1.b;
// check for button-press state transitions.
if ((bCurrStateB) && (bCurrStateB != bPrevStateB)) {
//imu.initialize(parameters);
v_state_current = 0;                // button is transitioning to a pressed state.  Reset the v_state
}
// update previous state variable.
bPrevStateB = bCurrStateB;
if (bCurrStateB) {
tri_state = 1;
}


// We can be doing 1 of 3 things
// tri_state = 0 means we are driving
// tri_state = 1 means we are capturing a beacon (uses color)
// tri_state = 2 means we are stealing a beacon (presses both sides)

if (tri_state == 1) {
*/
            switch (v_state[v_state_current]) {

                case 0:
                    break;

                case 1: // Drive stright (in any of 4 directions) for a given amount of time

                    robot.frontRightDrive.setPower(leftArray[v_state_current] + rightArray[v_state_current]);
                    robot.frontLeftDrive.setPower(leftArray[v_state_current] - rightArray[v_state_current]);
                    robot.rearRightDrive.setPower(leftArray[v_state_current] - rightArray[v_state_current]);
                    robot.rearLeftDrive.setPower(leftArray[v_state_current] + rightArray[v_state_current]);
                    intRunTime = runtime.milliseconds() + timeArray[v_state_current];
                    while (runtime.milliseconds() < intRunTime) {
                        telemetry.addData("drive-l", leftArray[v_state_current]);
                        telemetry.addData("drive-r", rightArray[v_state_current]);
                        telemetry.update();
                        idle();
                    }
                    robot.frontRightDrive.setPower(0.0);
                    robot.frontLeftDrive.setPower(0.0);
                    robot.rearRightDrive.setPower(0.0);
                    robot.rearLeftDrive.setPower(0.0);

                    v_state_current++;
                    break;

                case 2: // Drive straight (in any of 4 directions) for a given distance (encoder count)

                    encoderDrive(leftArray[v_state_current], rightArray[v_state_current], timeArray[v_state_current]);
                    v_state_current++;
                    break;

                case 3: // Turn using the IMU
                    while (opModeIsActive() && !onHeading(leftArray[v_state_current], rightArray[v_state_current], timeArray[v_state_current], P_TURN_COEFF)) {
                        // Update telemetry & Allow time for other processes to run
                        telemetry.update();
                        idle();
                    }
                    v_state_current++;
                    break;

                case 4: // Lower the bot
                    // Start by raising the hook
                    robot.collector.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.collector.setTargetPosition(3600);
                    robot.collector.setPower(1.0f);
                    while (opModeIsActive() && robot.collector.isBusy()) {
                        // Update telemetry & Allow time for other processes to run
                        telemetry.update();
                        idle();
                    }
                    // Now unlatch the hook
                    encoderDrive(0, 500, 0.3);
                    // Lower the hook
                    robot.collector.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.collector.setTargetPosition(0);
                    robot.collector.setPower(1.0f);

                    encoderDrive(-500, 0, 0.3);
                    //encoderDrive(200,0,0.3);
                    //encoderDrive(0,-200,0.3);

                    robot.collector.setPower(0.0f);
                    v_state_current++;
                    break;

                case 5: // Drive using the TimeOfFlight sensor
                    if (robot.sensorAxis != null) {
                        if ((leftArray[v_state_current] + rightArray[v_state_current]) * rightArray[v_state_current] == 0) {
                            servoPOS = servoPOS1;
                        } else {
                            servoPOS = servoPOS2;
                        }
                        if (Math.abs(robot.sensorAxis.getPosition() - servoPOS) > 0.05) {
                            robot.sensorAxis.setPosition(servoPOS);
                            if (robot.teamID == "15555") {
                                robot.collector.setTargetPosition(0);
                                robot.collector.setPower(1.0f);
                            }
                            sleep(500);
                            if (robot.teamID == "15555") robot.collector.setPower(0.0f);
                        }

                    }

                    while (opModeIsActive() && !onToF(leftArray[v_state_current], rightArray[v_state_current], timeArray[v_state_current], P_TURN_COEFF)) {
                        // Update telemetry & Allow time for other processes to run
                        telemetry.update();
                        idle();
                    }
                    v_state_current++;
                    break;

                case 6: // sensorAxis control
                    servoPOS1 = leftArray[v_state_current];
                    servoPOS2 = rightArray[v_state_current];
                    v_state_current++;
                    break;

                case 7: // push particle
                    NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
                    int color = colors.toColor();
                    float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                    colors.red /= max;
                    colors.green /= max;
                    colors.blue /= max;
                    color = colors.toColor();
                    if ((Color.red(color) * 100 / Color.blue(color)) > timeArray[v_state_current]) {
                        encoderDrive(0, 500, 0.2);
                        encoderDrive(0, -600, 0.2);
                        v_state_current += (int) (leftArray[v_state_current]);
                    }
                    v_state_current++;
                    break;

                case 8: // deliver team marker
                    robot.servoMarker.setPosition(0.5);
                    sleep(750);
                    v_state_current++;
                    break;

                case 12:
                    encoderDrive(3500, 0, 0.5);
                    robot.armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armExtend.setTargetPosition(75);
                    robot.armExtend.setPower(0.3f);
                    if (robot.armMove != null) {
                        robot.armMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (robot.teamID == "10645") {
                            robot.armMove.setTargetPosition(4500);
                        } else {
                            robot.armMove.setTargetPosition(5500);
                        }
                        robot.armMove.setPower(Math.abs(0.2));

                        while (opModeIsActive() && robot.armMove.isBusy()) {

                            float aePos = (int) (Range.clip(robot.armMove.getCurrentPosition(), 0, 5500) / 7.11);
                            robot.armExtend.setTargetPosition((int) aePos); // based on a double 15:40 tooth reduction setup
                            robot.armExtend.setPower(0.2f);

                        }
                    }

                    robot.armExtend.setTargetPosition(robot.armExtend.getCurrentPosition() - 75);
                    robot.armExtend.setPower(0.2f);
                    if (robot.teamID == "10645") {
                        robot.collector.setPower(-1);
                    } else {
                        robot.collector.setPower(1);
                    }
                    sleep(2000);
                    robot.collector.setPower(0);

                    robot.armMove.setTargetPosition(0);
                    while (opModeIsActive() && robot.armMove.isBusy()) {

                        float aePos = (int) (Range.clip(robot.armMove.getCurrentPosition(), 0, 5500) / 7.11);
                        robot.armExtend.setTargetPosition((int) aePos); // based on a double 15:40 tooth reduction setup
                        robot.armExtend.setPower(0.2f);

                    }

                    robot.armMove.setPower(Math.abs(0.3));


                    v_state_current++;
                    break;

                case 13:

                    robot.armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armExtend.setTargetPosition(75);
                    robot.armExtend.setPower(0.3f);
                    if (robot.armMove != null) {
                        robot.armMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        {
                            robot.armMove.setTargetPosition(5500);
                        }
                        robot.armMove.setPower(Math.abs(0.2));

                        while (opModeIsActive() && robot.armMove.isBusy()) {

                            float aePos = (int) (Range.clip(robot.armMove.getCurrentPosition(), 0, 5500) / 7.11);
                            robot.armExtend.setTargetPosition((int) aePos); // based on a double 15:40 tooth reduction setup
                            robot.armExtend.setPower(0.2f);

                        }
                    }

                    robot.armExtend.setTargetPosition(robot.armExtend.getCurrentPosition() - 75);
                    robot.armExtend.setPower(0.2f);
                {
                    robot.collector.setPower(-0.5);
                }
                sleep(2000);
                robot.collector.setPower(0);

                robot.armMove.setTargetPosition(0);
                robot.armMove.setPower(Math.abs(0.3));
                while (opModeIsActive() && robot.armMove.isBusy()) {

                    float aePos = (int) (Range.clip(robot.armMove.getCurrentPosition(), 0, 5500) / 7.11);
                    robot.armExtend.setTargetPosition((int) aePos); // based on a double 15:40 tooth reduction setup
                    robot.armExtend.setPower(0.2f);

                }

                v_state_current++;
                break;

                case 55:
                    /** Activate Tensor Flow Object Detection. */
                    if (tfod != null) {
                        tfod.activate();
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            //telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        leftArray[v_state_current] -= 25;
                                        rightArray[v_state_current] +=25;
                                        //telemetry.addData("Gold Mineral Position", "Left");
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        //telemetry.addData("Gold Mineral Position", "Right");
                                    } else {
                                        leftArray[v_state_current] += 25;
                                        rightArray[v_state_current] -=25;
                                        //telemetry.addData("Gold Mineral Position", "Center");
                                    }
                                }
                            }
                            //telemetry.update();
                        }
                        tfod.shutdown();

                    }
                    v_state_current++;
                    break;
            }

/*
case 2: // Arm Up & Slide Retract
armMotor.setPower(leftArray[v_state_current]);
slideMotor.setPower(rightArray[v_state_current]);
intRunTime = runtime.milliseconds() + timeArray[v_state_current];
while (runtime.milliseconds() < intRunTime) {
idle();
}
armMotor.setPower(0.0);
slideMotor.setPower(0.0);
v_state_current++;
break;

case 3:
jewelServo.setPosition(jewelPositionDn);
intRunTime = runtime.milliseconds() + 500;
while (runtime.milliseconds() < intRunTime) {
idle();
}
r = sensorRGB.red();
b = sensorRGB.blue();

// Default to positive for RED, negative for BLUE
n = 1;
if (OpModeName.toLowerCase().contains("blue")) {
n = -1;
}

// Change sign if RED is seen
if (r > b) {
n = -n;
}

// Do not touch jewels if no color is seen
if (b == r) {
n = 0;
}

leftMotor.setPower(n * leftArray[v_state_current]);
rightMotor.setPower(n * rightArray[v_state_current]);
intRunTime = runtime.milliseconds() + timeArray[v_state_current];
while (runtime.milliseconds() < intRunTime) {
idle();
}
leftMotor.setPower(0.0);
rightMotor.setPower(0.0);
jewelServo.setPosition(jewelPositionUp);

v_state_current++;
break;

case 4:

handOffset = 0.3;
if (rightServo != null) rightServo.setPosition(handPosition - handOffset);
if (leftServo != null) leftServo.setPosition(handPosition + handOffset);

// Request a mark found variable update from the camera app.
RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
intRunTime = runtime.milliseconds() + timeArray[v_state_current];
while (runtime.milliseconds() < intRunTime) {
idle();
}
CryptoBoxOffset = 0;  // Corresponds to the CENTER column since our driving approach simply alters the distance travelled in front of the cryptobox.   Decrease for near column, increase for far column

// Asume RED where right column means decrease
// CryptoBoxKey found; Update the target column & offset distance.
if (vuMark == RelicRecoveryVuMark.LEFT) {
CryptoBoxOffset = (int) leftArray[v_state_current]; // Using our +1 / -1 toggle for jewel selection to offset the driving distance - this works because the field is mirror imaged as well
}
if (vuMark == RelicRecoveryVuMark.RIGHT) {
CryptoBoxOffset = -(int) leftArray[v_state_current]; // Using our +1 / -1 toggle for jewel selection to offset the driving distance  - this works because the field is mirror imaged as well
}
//if (OpModeName.toLowerCase().contains("blue")) {
//    CryptoBoxOffset  = -CryptoBoxOffset ;
//}

v_state_current++;
break;

case 5:
handOffset = 0.1;
if (rightServo != null) rightServo.setPosition(handPosition - handOffset);
if (leftServo != null) leftServo.setPosition(handPosition + handOffset);
v_state_current++;
break;

case 6:
intRunTime = runtime.milliseconds() + timeArray[v_state_current];
while (runtime.milliseconds() < intRunTime) {
idle();
}
v_state_current++;
break;

case 7:

jewelServo.setPosition(jewelPositionDn);
intRunTime = runtime.milliseconds() + 600;
while (runtime.milliseconds() < intRunTime) {
idle();
}
r = sensorRGB.red();
b = sensorRGB.blue();

intRunTime = runtime.milliseconds() + 75;
while (runtime.milliseconds() < intRunTime) {
idle();
}
n = sensorRGB.red();
if (n > r) {
r = n;
}
n = sensorRGB.blue();
if (n > b) {
b = n;
}

intRunTime = runtime.milliseconds() + 75;
while (runtime.milliseconds() < intRunTime) {
idle();
}
n = sensorRGB.red();
if (n > r) {
r = n;
}
n = sensorRGB.blue();
if (n > b) {
b = n;
}

// Default to positive for RED, negative for BLUE
n = 1;
if (OpModeName.toLowerCase().contains("red")) {
n = -1;
}

// Change sign if RED is seen
if (r > b) {
n = -n;
}

// Do not touch jewels if no color is seen
if (b == r) {
n = 0;
}

// call GyroTurn (n * 10) to turn either +10 or -10 degrees
gyroTurn(0.0, 0.25, n * 10);
jewelServo.setPosition(jewelPositionUp);
// call GyruTurn (0)
gyroTurn(0.0, 0.25, 0);         // Return to 0 degrees

v_state_current++;
break;
case 8:
int newLeftTarget = leftMotor.getCurrentPosition() + (int) timeArray[v_state_current];
int newRightTarget = rightMotor.getCurrentPosition() + (int) timeArray[v_state_current];

// Set Target and Turn On RUN_TO_POSITION
leftMotor.setTargetPosition(newLeftTarget);
rightMotor.setTargetPosition(newRightTarget);

leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


// start motion.
//timeArray[v_state_current] = Range.clip(Math.abs(timeArray[v_state_current]), 0.0, 1.0);
leftMotor.setPower(leftArray[v_state_current]);
rightMotor.setPower(rightArray[v_state_current]);

// keep looping while we are still active, and BOTH motors are running.
while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy())) {
idle();
}

// Stop all motion;
leftMotor.setPower(0);
rightMotor.setPower(0);

// Turn off RUN_TO_POSITION
leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

v_state_current++;
break;
case 9:
gyroTurn(leftArray[v_state_current], rightArray[v_state_current], timeArray[v_state_current]);         // Turn 10 degrees to know jewel
v_state_current++;
break;
case 10:
int newLeftTarget1 = leftMotor.getCurrentPosition() + (int) timeArray[v_state_current] + CryptoBoxOffset;
int newRightTarget1 = rightMotor.getCurrentPosition() + (int) timeArray[v_state_current] + CryptoBoxOffset;

// Set Target and Turn On RUN_TO_POSITION
leftMotor.setTargetPosition(newLeftTarget1);
rightMotor.setTargetPosition(newRightTarget1);

leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


// start motion.
//timeArray[v_state_current] = Range.clip(Math.abs(timeArray[v_state_current]), 0.0, 1.0);
leftMotor.setPower(leftArray[v_state_current]);
rightMotor.setPower(rightArray[v_state_current]);

// keep looping while we are still active, and BOTH motors are running.
while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy())) {
idle();
}

// Stop all motion;
leftMotor.setPower(0);
rightMotor.setPower(0);

// Turn off RUN_TO_POSITION
leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

v_state_current++;
break;
/
case 11:
int newDumpTarget = dumpMotor.getCurrentPosition() + (int) timeArray[v_state_current];
dumpMotor.setTargetPosition(newDumpTarget);
dumpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

dumpMotor.setPower(leftArray[v_state_current]);
// keep looping while we are still active, and BOTH motors are running.
while (opModeIsActive() && dumpMotor.isBusy()) {
idle();
}

// Stop all motion;
dumpMotor.setPower(0);

// Turn off RUN_TO_POSITION
dumpMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
v_state_current++;
break;

case 12:
//int newliftTarget = liftMotor.getCurrentPosition() + (int) timeArray[v_state_current];
//liftMotor.setTargetPosition(newliftTarget);
//liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//liftMotor.setPower(leftArray[v_state_current]);
// keep looping while we are still active, and BOTH motors are running.
//while (opModeIsActive() && liftMotor.isBusy()) {
//    idle();
//}

// Stop all motion;
//liftMotor.setPower(0);

// Turn off RUN_TO_POSITION
//liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
v_state_current++;
break;

}
}

if (tri_state == 2) {
settings.newSettings();
v_state_current = 0;

while (v_state[v_state_current] > 0) {
settings.SetIntSetting(String.format("v_state%02d", v_state_current), v_state[v_state_current]);
settings.SetSetting(String.format("timeArray%02d", v_state_current), timeArray[v_state_current]);
settings.SetSetting(String.format("rightArray%02d", v_state_current), rightArray[v_state_current]);
settings.SetSetting(String.format("leftArray%02d", v_state_current), leftArray[v_state_current]);
v_state_current++;
}
settings.SetIntSetting("v_state_count", v_state_current);
settings.SaveSetting(OpModeName);

}

if (tri_state == 0) {

mdelta = 0;
left = 0;
right = 0;

bCurrStateY = gamepad1.y;
// check for button-press state transitions.
if ((bCurrStateY) && (bCurrStateY != bPrevStateY)) {
mdelta = 1;
left = 10;
right = 0.1;
} // button is transitioning to a pressed state.  Reset the v_state
// update previous state variable.
bPrevStateY = bCurrStateY;

//                bCurrStateX = gamepad1.x;
//                // check for button-press state transitions.
//                if ((bCurrStateX) && (bCurrStateX != bPrevStateX)) { delta = -1; left = -10; right=-0.1;} // button is transitioning to a pressed state.  Reset the v_state
//                // update previous state variable.
//                bPrevStateX = bCurrStateX;

bCurrStateA = gamepad1.a;
// check for button-press state transitions.
if ((bCurrStateA) && (bCurrStateA != bPrevStateA)) {
mdelta = -1;
left = -10;
right = -0.1;
} // button is transitioning to a pressed state.
// update previous state variable.
bPrevStateA = bCurrStateA;

//                bCurrStateB = gamepad1.b;
//                // check for button-press state transitions.
//                if ((bCurrStateB) && (bCurrStateB != bPrevStateB)) { calibration = -1; right = -1; left=-0.1;} // button is transitioning to a pressed state.
//                bPrevStateB = bCurrStateB;

bCurrdpadUP = gamepad1.dpad_up;
// check for button-press state transitions.
if ((bCurrdpadUP) && (bCurrdpadUP != bPrevdpadUP)) {
v_state_current++;
} // button is transitioning to a pressed state.
// update previous state variable.
bPrevdpadUP = bCurrdpadUP;

bCurrdpadDOWN = gamepad1.dpad_down;
// check for button-press state transitions.
if ((bCurrdpadDOWN) && (bCurrdpadDOWN != bPrevdpadDOWN)) {
v_state_current--;
} // button is transitioning to a pressed state.
bPrevdpadDOWN = bCurrdpadDOWN;

bCurrdpadLEFT = gamepad1.dpad_left;
// check for button-press state transitions.
if ((bCurrdpadLEFT) && (bCurrdpadLEFT != bPrevdpadLEFT)) {
dpad--;
} // button is transitioning to a pressed state.
bPrevdpadLEFT = bCurrdpadLEFT;

bCurrdpadRIGHT = gamepad1.dpad_right;
// check for button-press state transitions.
if ((bCurrdpadRIGHT) && (bCurrdpadRIGHT != bPrevdpadRIGHT)) {
dpad++;
} // button is transitioning to a pressed state.
bPrevdpadRIGHT = bCurrdpadRIGHT;

if (dpad < 1) {
dpad = 1;
}
if (dpad > 4) {
dpad = 4;
}
if (v_state_current < 0) {
v_state_current = 0;
}
if (v_state_current > ArraySize) {
v_state_current = ArraySize;
}

multiplier1 = 1;
if (gamepad1.left_bumper) {
multiplier1 = 10;
}
if (gamepad1.right_bumper) {
multiplier1 = 0.5;
}

// slow move (driver control)

left = left * multiplier1;
right = right * multiplier1;

up = up * multiplier2;
down = down * multiplier2;

if (dpad == 1) {
v_state[v_state_current] = v_state[v_state_current] + mdelta;
}
if (dpad == 2) {
timeArray[v_state_current] = timeArray[v_state_current] + left;
}
if (dpad == 3) {
rightArray[v_state_current] = rightArray[v_state_current] + right;
}
if (dpad == 4) {
leftArray[v_state_current] = leftArray[v_state_current] + right;
}
//if (timeArray[v_state_current] < -180) {timeArray[v_state_current] = 0; }
if (leftArray[v_state_current] < -1) {
leftArray[v_state_current] = -1;
}
if (leftArray[v_state_current] > 1) {
leftArray[v_state_current] = 1;
}
if (rightArray[v_state_current] < -1) {
rightArray[v_state_current] = -1;
}
if (rightArray[v_state_current] > 1) {
rightArray[v_state_current] = 1;
}
}

telemetry.addData("red / blue / n ", r + " / " + b + " / " + n);


telemetry.addData("v_state_current / dpad / tristate ", (v_state_current + " /  " + dpad + " / " + tri_state));
telemetry.addData("v_state / timeArray ", (v_state[v_state_current] + " / " + timeArray[v_state_current]));
telemetry.addData("leftArray / rightArray ", (leftArray[v_state_current] + " / " + rightArray[v_state_current]));
telemetry.addData("tristate ", (tri_state));
telemetry.update();

idle();

}
*/
            idle();
        }

    }

    boolean onHeading(double leftspeed, double rightspeed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftspeed = 0.0;
            rightspeed = 0.0;
            onTarget = true;
        } else {
            steer = 1.0;
            if (Math.abs(error) <= HEADING_THRESHOLD * 5) { steer = 0.1 / (Math.abs(leftspeed) + Math.abs(rightspeed)); }
            leftspeed = leftspeed * steer;
            rightspeed = rightspeed * steer;
        }

        // Send desired speeds to motors.
        robot.frontRightDrive.setPower(-leftspeed);
        robot.frontLeftDrive.setPower(+leftspeed);
        robot.rearRightDrive.setPower(-leftspeed);
        robot.rearLeftDrive.setPower(+leftspeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftspeed, rightspeed);

        return onTarget;
    }
    boolean onColor(double leftspeed, double rightspeed,double distance) {
        boolean onTarget = false;
        if (String.format(Locale.US, "%.02f", robot.colorRange.getDistance(DistanceUnit.CM))=="NaN") {
            onTarget = false;
        } else if (robot.colorRange.getDistance(DistanceUnit.CM) > distance) {
            onTarget = false;
        } else {
            onTarget = true;
        }
        if (!onTarget) { encoderDrive(leftspeed,rightspeed,0.1);}
        return onTarget;



    }
    boolean onToF(double leftspeed, double forwardspeed, double distance, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;

        // determine turn power based on +/- error
        error = getToFerror(distance);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftspeed = 0.0;
            forwardspeed = 0.0;
            onTarget = true;
        } else {
            steer = 1.0;
            if (Math.abs(error) <= HEADING_THRESHOLD * 5) { steer = 0.1 / (Math.abs(leftspeed) + Math.abs(forwardspeed)); }
            leftspeed = leftspeed * steer * -(error)/Math.abs((error));
            forwardspeed = forwardspeed * steer * -(error)/Math.abs((error));
        }

        // Send desired speeds to motors.
        robot.frontRightDrive.setPower(leftspeed + forwardspeed );
        robot.frontLeftDrive.setPower(leftspeed - forwardspeed);
        robot.rearRightDrive.setPower(leftspeed - forwardspeed);
        robot.rearLeftDrive.setPower(leftspeed + forwardspeed );

        // Display it for the driver
        telemetry.addData("Target", "%5.2f", distance);
        telemetry.addData("Err/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftspeed, forwardspeed);

        return onTarget;
    }


    /*
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */

    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getToFerror(double targetDistance) {

        double robotError;
        robotError = targetDistance - robot.sensorRange.getDistance(DistanceUnit.CM);
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void encoderDrive(double countx, double county, double speed) {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int FrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(countx + county);
        int FrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(countx - county);
        int RearRightTarget = robot.rearRightDrive.getCurrentPosition() + (int)(countx - county);
        int RearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + (int)(countx + county);

        robot.frontRightDrive.setTargetPosition(FrontRightTarget);
        robot.frontLeftDrive.setTargetPosition(FrontLeftTarget);
        robot.rearRightDrive.setTargetPosition(RearRightTarget);
        robot.rearLeftDrive.setTargetPosition(RearLeftTarget);
        // start motion
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
        robot.rearRightDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        // keep looping while we are still active, and motors are running
        while (opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
            telemetry.update();
            idle();
        }
        // Stop all motion;
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

/*
public void gyroTurn(double leftspeed, double rightspeed, double angle) {

// keep looping while we are still active, and not on heading.
while (opModeIsActive() && !onHeading(leftspeed, rightspeed, angle, P_TURN_COEFF)) {
// Update telemetry & Allow time for other processes to run.
telemetry.update();
}

/**
* returns desired steering force.  +/- 1 range.  +ve = steer left
*
* @param error  Error angle in robot relative degrees
* @param PCoeff Proportional Gain Coefficient
* @return
*/





}

