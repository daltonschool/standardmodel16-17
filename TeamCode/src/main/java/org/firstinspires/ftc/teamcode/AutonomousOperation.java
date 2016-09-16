/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.view.Gravity;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous Operation")
public class AutonomousOperation extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor rightBackMotor = null;
    private ColorSensor colorSensor = null;
    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Acceleration gravity = null;
    private float distanceDriven = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // motors
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        leftBackMotor  = hardwareMap.dcMotor.get("left back");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        rightBackMotor = hardwareMap.dcMotor.get("right back");

        // motor directions
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // color sensor
        //colorSensor = hardwareMap.colorSensor.get("color sensor");

        // imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

    public void updateImu() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        telemetry.addData("heading", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        gravity = imu.getGravity();
        telemetry.addLine()
                .addData("grvty", gravity.toString())
                .addData("mag", String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel)));


        telemetry.addData("distanceDriven", distanceDriven);
        telemetry.update();
    }

    public void updateSensors() {
        // update imu info
        updateImu();
    }

    public void leftMotors(double power) {
        leftMotor.setPower(power);
        leftBackMotor.setPower(power);
    }

    public void rightMotors(double power) {
        rightMotor.setPower(power);
        rightBackMotor.setPower(power);
    }

    public void turnToHeading(float targetHeading, double power) {
        float currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        boolean turnLeft = (targetHeading - currentHeading > 0 ? true : false);
        while (true) {
            double currentSpeed = power;
            float distanceTo = Math.abs(targetHeading - currentHeading);

            if (distanceTo < 10) {
                currentSpeed *= 0.10;
            } else if (distanceTo < 20) {
                currentSpeed *= 0.25;
            } else if (distanceTo < 30) {
                currentSpeed *= 0.50;
            }

            leftMotors((turnLeft ? -currentSpeed : currentSpeed));
            rightMotors((turnLeft ? currentSpeed : -currentSpeed));

            updateImu();
            currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

            if (turnLeft && targetHeading < currentHeading) {
                break;
            } else if (!turnLeft && targetHeading > currentHeading) {
                break;
            }
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }

    public void moveForward(double distanceToDrive, double power) {
        distanceDriven = 0.0f;
        while (distanceDriven < (distanceToDrive * 100)) {
            leftMotors(power);
            rightMotors(power);
            updateImu();
            distanceDriven += Math.abs(gravity.xAccel);
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }


    public int step = 0;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        updateSensors();

        //leftMotor.setPower(0.25);
        //rightMotor.setPower(0.25);

        switch (step) {
            case 0:
                moveForward(5, 0.20);
                break;
            case 1:
                turnToHeading(160, 0.15);
                distanceDriven = 0.0f;
                break;
            case 2:
                moveForward(5, 0.20);
                break;
            case 3:
                turnToHeading(20, 0.15);
                break;
        }
        step++;
        //turnToHeading(0, 0.20);
        //turnToHeading(-90, 0.20);
        //requestOpModeStop();

        /*telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());*/
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
