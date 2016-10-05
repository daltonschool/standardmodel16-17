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

import android.graphics.Bitmap;
import android.util.Log;
import android.view.Gravity;
import android.widget.Toast;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Vuforia;
import com.vuforia.ar.pl.DebugLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.ClassFactory_SM;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.VuforiaLocalizerImpl_SM;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
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
public class AutonomousOperation extends LinearOpMode
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
    private VuforiaLocalizer vuforia;
    private List<VuforiaTrackable> allTrackables;

    private OpenGLMatrix lastLocation = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

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
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled      = true;
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        // vuforia init
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        vuforiaParameters.vuforiaLicenseKey = "Aeqqx9n/////AAAAGU44IlIke0wcpp2TXZIm0doq2mr4uV5sFkonVd69btVkAHlcthh2lKkMMI+n0pvfyHG/1YVon/+hvr2sJ14bJp3HFifDm0EDP1lJ0B26oSFaShv339Snwjk53VLnXiIAxRu6ys9uovyitz8dlnnT8j6UHSRV1elViHriLiSJt9URKaUhoe0I0a+0XElImXIuZXN7p8NMMP/LIPK3bHYt3CIMIGQ4fSs1+4/06pqI06ijwsH1SIIZn0tiB4199YwyqLfea3Wi+Tsnwm3IkOhgWCy3JeHiCTs43EmciCH0ldF+2N/XuoFFMMPqe/81vMhdHWuWuQFPtXDK7wYrLNFqZ32YTGyKkhyFaejloP4No76F";
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory_SM.createVuforiaLocalizer_SM(vuforiaParameters);

        // vuforia trackables
        VuforiaTrackables ftcTrackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable wheels = ftcTrackables.get(0);
        wheels.setName("Wheels");

        VuforiaTrackable tools = ftcTrackables.get(1);
        tools.setName("Tools");

        VuforiaTrackable legos = ftcTrackables.get(2);
        legos.setName("Legos");

        VuforiaTrackable gears = ftcTrackables.get(3);
        gears.setName("Gears");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ftcTrackables);

        // units
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 15.5f * mmPerInch;
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        // place beacons on field
        OpenGLMatrix gearsLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, -(2*12*mmPerInch), 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(gearsLocationOnField);

        OpenGLMatrix toolsLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, (2*12*mmPerInch), 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(toolsLocationOnField);

        OpenGLMatrix legosLocationOnField = OpenGLMatrix
                .translation(-(2*12*mmPerInch), mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(legosLocationOnField);

        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix
                .translation((2*12*mmPerInch), mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(wheelsLocationOnField);

        // phone location on robot
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));

        // inform listeners about phone
        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);

        // ready to go!
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // activate vuforia dataset
        ftcTrackables.activate();

        while (true) {
            telemetry.addData("Status", "Running: " + runtime.toString());
            updateSensors();

            updateVuforia();

            if (lastLocation != null) {
                telemetry.addData("Pos", lastLocation.formatAsTransform());
            } else {
                telemetry.addData("Pos", "I am lost :(");
            }

            //leftMotor.setPower(0.25);
            //rightMotor.setPower(0.25);

            alignToTrackable(allTrackables.indexOf(gears));

            telemetry.update();
            //turnToHeading(0, 0.20);
            //turnToHeading(-90, 0.20);
            //requestOpModeStop();

            /*telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());*/
        }
    }

    public void updateVuforia() {
        // vuforia updating
        for (VuforiaTrackable trackable : allTrackables) {
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
            //((VuforiaTrackableDefaultListener)trackable.getListener()).
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
    }

    public void alignToTrackable(int trackIndex) {
        while (true) {
            if (lastLocation == null) {
                updateVuforia();
                continue;
            }
            VuforiaTrackable track = allTrackables.get(trackIndex);
            OpenGLMatrix m = lastLocation;
            VectorF translation = m.getTranslation();
            Orientation orientation = Orientation.getOrientation(m, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            boolean orientationGood = ((orientation.thirdAngle > -160 && orientation.thirdAngle < -180) ||
                                        (orientation.thirdAngle > 160 && orientation.thirdAngle < 180));
            if (!orientationGood && orientation.thirdAngle < 0) {
                leftMotors(0.1f);
                rightMotors(0.05f);
            } else if (!orientationGood && orientation.thirdAngle > 0) {
                leftMotors(0.05f);
                rightMotors(0.1f);
            } else if (translation.get(0) > -900) {
                //moveForward(Math.abs((-1100 - translation.get(0)) / 4), 0.2f);
                leftMotors(0.1f);
                rightMotors(0.1f);
            } else {
                leftMotors(0.0f);
                rightMotors(0.0f);
                return;
            }
            updateVuforia();

            telemetry.addData("Pos", lastLocation.formatAsTransform());
            telemetry.addData("Ptest", translation.get(0));
            telemetry.addData("Test", leftMotor.getCurrentPosition());
            telemetry.update();
        }
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
        //telemetry.update();
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

}