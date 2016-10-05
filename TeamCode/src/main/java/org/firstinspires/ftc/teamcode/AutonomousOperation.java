package org.firstinspires.ftc.teamcode;

import java.util.Date;
import android.graphics.Color;
import android.view.Gravity;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.text.SimpleDateFormat;

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
    private ColorSensor colorSensor = null;

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
        colorSensor = hardwareMap.colorSensor.get("color sensor");

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

    public void lineFollower() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        double whiteLineIntensityValue = 23.6; //Replace with value that we get
        double blackMatIntensityValue = 1.24;  //Also do that here
        double targetIntensity = whiteLineIntensityValue + blackMatIntensityValue / 2; //Get tarrget value

        while (true) //change to run until called
        {
            if (Math.abs(hsvValues[2] - targetIntensity) <= 5) //if on line
            {
                moveForward(1, .3);
            } else {
                int i = 0;
                while (hsvValues[2] - targetIntensity > 5 || i < 90) {
                    turnToHeading((AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + 5), .3);
                    i++;
                }
                turnToHeading((AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) - (i * 5)), .3);
                while (hsvValues[2] - targetIntensity > 5 || i < 90) {
                    turnToHeading((AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) - 5), .3);
                }
            }
//            else
//            {
//                System.out.println("3Error");
//                break;
//            }
        }
    }

}

public class PIDcontroller {

    //Date previous_time = new Date();
    double total_integral = 0;
    double P = .15;
    double I = 0;
    double D = 0;
    //long d1 = previous_time.getTime();
    long previous_time =  System.nanoTime();

    public double Step( double cError ) {
        double current_error = cError;
        //Date end = new Date();
        //long d2 = end.getTime();
        long end =  System.nanoTime();
        double time_diff = (end - previous_time) / 1000;
        double error_diff = current_error * time_diff;
        double integral_step = current_error * time_diff;
        total_integral = total_integral + integral_step;

        double Kp = current_error;
        if (Kp < 0) {
            Kp *= 1.8;
        }
        double Ki = total_integral;
        double Kd = error_diff / time_diff;

        double R = P*Kp + I*Ki + D*Kd;
        if (R > 100) {
            R = 100;
        }
        else if (R < -100) {
            R = -100;
        }
        System.out.println("Kp: " + (Kp * P));
        System.out.println("Ki: " + (Ki * I));
        System.out.println("Kd: " + (Kd * D);
        System.out.println("R: " + R);
        System.out.println("Current Error" + current_error);

        return R;
    }
}