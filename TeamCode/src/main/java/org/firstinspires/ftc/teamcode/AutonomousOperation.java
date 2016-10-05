package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name="Autonomous Operation")
public class AutonomousOperation extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor rightBackMotor = null;
    private IMU imu = null;
    private float distanceDriven = 0;

    private Alliance currentAlliance = Alliance.RED; // this is hardcoded for now, but should be set somehow (two different opmodes? how is this normally done?)
    private Vuforia vuforia;

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
        imu = new IMU();
        imu.init(hardwareMap.get(BNO055IMU.class, "imu"));

        vuforia = new Vuforia();
        vuforia.init();

        // ready to go!
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        vuforia.start();

        while (true) {
            telemetry.addData("Status", "Running: " + runtime.toString());

            // update sensors
            imu.update();
            vuforia.update();

            if (vuforia.hasLocation()) {
                telemetry.addData("Pos", vuforia.getLocationAsString());
            } else {
                telemetry.addData("Pos", "I am lost :(");
            }

            //leftMotor.setPower(0.25);
            //rightMotor.setPower(0.25);

            alignTest();

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

    public void alignTest() { // only works for gears!!!
        while (true) {
            if (!vuforia.hasLocation()) {
                vuforia.update();
                continue;
            }
            VectorF translation = vuforia.getLocation();
            Orientation orientation = vuforia.getOrientation();
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
            vuforia.update();

            telemetry.addData("Pos", vuforia.getLocationAsString());
            telemetry.addData("Ptest", translation.get(0));
            telemetry.addData("Test", leftMotor.getCurrentPosition());
            telemetry.update();
        }
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
        float currentHeading = imu.getHeading();
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

            imu.update();
            currentHeading = imu.getHeading();

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
            imu.update();
            distanceDriven += Math.abs(imu.getGravity().xAccel);
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }

}