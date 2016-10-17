package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Autonomous Operation")
public class AutonomousOperation extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftMotor = null;
    //private DcMotor leftBackMotor = null;
    private DcMotor rightMotor = null;
    //private DcMotor rightBackMotor = null;
    private IMU imu = null;
    private float distanceDriven = 0;
    private ColorSensor colorSensor = null;
    private Servo beaconServo = null;
    private ColorSensor beaconColor = null;

    private Alliance currentAlliance = Alliance.RED; // this is hardcoded for now, but should be set somehow (two different opmodes? how is this normally done?)
    private Vuforia vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        // motors
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        //leftBackMotor  = hardwareMap.dcMotor.get("left back");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        //rightBackMotor = hardwareMap.dcMotor.get("right back");

        // motor directions
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //leftBackMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //rightBackMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // servos
        beaconServo = hardwareMap.servo.get("beacon servo");

        // color sensor
        //colorSensor = hardwareMap.colorSensor.get("color sensor");
        beaconColor = hardwareMap.colorSensor.get("beacon color");
        beaconColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        beaconColor.enableLed(false);

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

            moveForward(18, 0.13f);
            telemetry.addLine("MOVE DONE");
            telemetry.update();
            Thread.sleep(3000);

            turnToHeading(70, 0.2f);
            telemetry.addLine("TURN DONE");
            telemetry.update();
            Thread.sleep(3000);

            moveForward(10, 0.13f);
            telemetry.addLine("MOVE PART 2 DONE");
            telemetry.update();
            Thread.sleep(3000);

            // align to gears
            telemetry.addLine("TRYING TO ALIGN");
            telemetry.update();
            alignTest();
            Thread.sleep(3000);

            telemetry.addLine("BEACON COLOR");
            telemetry.update();
            Thread.sleep(3000);

            for (int i = 0; i < 1500; i++) {
                telemetry.addData("r", beaconColor.red());
                telemetry.addData("g", beaconColor.green());
                telemetry.addData("b", beaconColor.blue());
                telemetry.update();
                Thread.sleep(1);
            }

            // determine beacon color
            Alliance leftColor = getBeaconLeftColor();
            if (leftColor == currentAlliance) {
                telemetry.addLine("PRESSING LEFT");
                beaconServo.setPosition(0.1);
            } else {
                telemetry.addLine("PRESSING RIGHT");
                beaconServo.setPosition(1.0);
            }
            telemetry.update();
            Thread.sleep(3000);

            telemetry.addLine("THE END");
            telemetry.update();

            // press button
            requestOpModeStop();

            telemetry.update();
            idle();
        }
    }

    public void alignTest() throws InterruptedException { // only works for gears!!!
        while (true) {
            if (!vuforia.hasLocation()) {
                vuforia.update();
                continue;
            }
            VectorF translation = vuforia.getLocation();
            Orientation orientation = vuforia.getOrientation();
            boolean orientationGood = ((orientation.thirdAngle > -160 && orientation.thirdAngle < -180) ||
                                        (orientation.thirdAngle > 160 && orientation.thirdAngle < 180));

            if (translation.get(0) < -1360) {
                // we're too close, just stop
                telemetry.addLine("ALIGNTEST IS DONE");
                leftMotors(0.0f);
                rightMotors(0.0f);
                return;
            }

            if (!orientationGood && orientation.thirdAngle < 0) {
                leftMotors(0.1f);
                rightMotors(0.05f);
            } else if (!orientationGood && orientation.thirdAngle > 0) {
                leftMotors(0.05f);
                rightMotors(0.1f);
            } else if (translation.get(0) > -1360) {
                //moveForward(Math.abs((-1100 - translation.get(0)) / 4), 0.2f);
                leftMotors(0.1f);
                rightMotors(0.1f);
            } else {
                telemetry.addLine("ALIGNTEST IS DONE");
                leftMotors(0.0f);
                rightMotors(0.0f);
                return;
            }
            vuforia.update();

            telemetry.addData("Pos", vuforia.getLocationAsString());
            telemetry.addData("Ptest", translation.get(0));
            telemetry.addData("Still going", (translation.get(0) > -1400));
            telemetry.addData("Test", leftMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }

    public void leftMotors(double power) {
        leftMotor.setPower(power);
        //leftBackMotor.setPower(power);
    }

    public void rightMotors(double power) {
        rightMotor.setPower(power);
        //rightBackMotor.setPower(power);
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

    public void moveForward(double distanceToDrive, double power) throws InterruptedException {
        distanceDriven = 0.0f;
        while (distanceDriven < (distanceToDrive * 100)) {
            leftMotors(power);
            rightMotors(power);
            imu.update();
            idle();
            distanceDriven += Math.abs(imu.getGravity().xAccel);
            telemetry.addData("distanceDriven", distanceDriven);
            telemetry.update();
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }

    public void lineFollower() throws InterruptedException {

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
                    turnToHeading((imu.getHeading() + 5), .3);
                    i++;
                }
                turnToHeading((imu.getHeading() - (i * 5)), .3);
                while (hsvValues[2] - targetIntensity > 5 || i < 90) {
                    turnToHeading((imu.getHeading() - 5), .3);
                    i++;
                }
            }
//            else
//            {
//                System.out.println("3Error");
//                break;
//            }
        }
    }

    public Alliance getBeaconLeftColor() throws InterruptedException {
        beaconServo.setPosition(0.45);
        telemetry.addLine("WAITING FOR SERVO POS");
        telemetry.update();
        Thread.sleep(5000);
        Alliance response;
        if (beaconColor.red() != 255 && beaconColor.red() > 100) {
            response = Alliance.RED;
        } else {
            response = Alliance.BLUE;
        }
        return response;
    }

}
