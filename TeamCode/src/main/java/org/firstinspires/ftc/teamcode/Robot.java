package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.IMU;
import org.firstinspires.ftc.teamcode.sensors.MRColorSensor;
import org.firstinspires.ftc.teamcode.sensors.PhoneGyro;
import org.firstinspires.ftc.teamcode.sensors.Sensor;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;

public class Robot {
    // Constants
    public static final double ODS_BLACK_VALUE = 0.08; // TODO: find this
    public static final double ODS_GRAY_VALUE = 0.1; // TODO: and this

    // Motors
    public static DcMotor leftMotor;
    public static DcMotor rightMotor;

    public static DcMotor flywheelLeft;
    public static DcMotor flywheelRight;

    public static DcMotor nom;

    public static DcMotor conveyor;

    // Servos
    public static Servo beaconLeft;
    public static Servo beaconRight;

    // Sensors
    public static ColorSensor leftBeaconColor = null;
    public static ColorSensor rightBeaconColor = null;
    public static ModernRoboticsI2cRangeSensor range = null;

    public static OpticalDistanceSensor leftLineLight;
    public static OpticalDistanceSensor rightLineLight;
    public static OpticalDistanceSensor centerLineLight;

    public static IMU imu = null;
    public static PhoneGyro phoneGyro = null;

    public static Vuforia vuforia;

    public static VoltageSensor voltageSensor;

    // Sensor list
    public static ArrayList<Sensor> sensors;

    // Telemetry
    public static Telemetry telemetry;

    // The original opmode.
    public static LinearOpMode opMode;

    // The app context
    public static Context appContext;

    // The current alliance
    public static Alliance currentAlliance;

    //Runtime
    ElapsedTime runtime = new ElapsedTime();

    public static void init(LinearOpMode om) {
        opMode = om;
        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        appContext = hardwareMap.appContext;
        sensors = new ArrayList<Sensor>();

        // Motors
        leftMotor = hardwareMap.dcMotor.get("drive_right");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        rightMotor = hardwareMap.dcMotor.get("drive_left");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        flywheelLeft = hardwareMap.dcMotor.get("launch_left");
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);

        flywheelRight = hardwareMap.dcMotor.get("launch_right");
        flywheelRight = hardwareMap.dcMotor.get("launch_right");
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        nom = hardwareMap.dcMotor.get("nom");
        nom.setDirection(DcMotor.Direction.REVERSE);

        conveyor = hardwareMap.dcMotor.get("lift");
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        // Servos
        beaconLeft = hardwareMap.servo.get("leftBeacon");
        beaconRight = hardwareMap.servo.get("rightBeacon");

        // Sensors
        // Color
        leftBeaconColor = hardwareMap.colorSensor.get("left beacon color");
        leftBeaconColor.enableLed(false);

        rightBeaconColor = hardwareMap.colorSensor.get("right beacon color");
        rightBeaconColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        rightBeaconColor.enableLed(false);

        // Optical distance
        leftLineLight = hardwareMap.opticalDistanceSensor.get("left_line");
        rightLineLight = hardwareMap.opticalDistanceSensor.get("right_line");
        centerLineLight = hardwareMap.opticalDistanceSensor.get("center_line");

        // IMU
        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        sensors.add(imu);

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        // Phone gyroscope
        //phoneGyro = new PhoneGyro();
        //sensors.add(phoneGyro);

        // Front distance
        //frontDist = hardwareMap.opticalDistanceSensor.get("front dist");

        // Vuforia
        vuforia = new Vuforia();
        //sensors.add(vuforia);

        // Voltage
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //Runtime

    }

    public static void start() {
        vuforia.start();
    }

    public static void update() {
        // update sensors
        for (Sensor s : sensors) {
            s.update();
        }

        // and telemetry
        telemetry.update();
    }

    public static void idle() throws InterruptedException {
        opMode.idle();
    }

    // Sensor management
    public static void verifyAllSensors() throws InterruptedException {
        Blackbox.log("SENSOR", "=== Sensor map ===");
        telemetry.addData("Status", "Starting sensors...");
        telemetry.update();
        int passedSensors = 0;
        int failedSensors = 0;
        int currentSensor = 1;
        ArrayList<String> failures = new ArrayList<String>();
        for (Sensor s : Robot.sensors) {
            telemetry.addData("Status", "Starting sensor " + currentSensor + " out of " + Robot.sensors.size() + "...");
            telemetry.update();
            Blackbox.log("SENSOR", s.uniqueName());
            Blackbox.log("SENSOR", "FW: " + Utils.intToHexString(s.firmwareRevision()) + ", MFG: " + Utils.intToHexString(s.manufacturer()) + ", CODE: " + Utils.intToHexString(s.sensorIDCode()));
            if (s.ping()) {
                // yay
                s.init();
                Blackbox.log("SENSOR", "PASS");
                s.update();
                passedSensors++;
            } else {
                // uh oh
                Blackbox.log("SENSOR", "FAIL");
                failures.add(s.uniqueName());
                failedSensors++;
            }
            currentSensor++;
        }

        Blackbox.log("SENSOR", passedSensors + " sensor(s) passed / " + failedSensors + " sensor(s) failed");

        if (failedSensors > 0) {
            // oh no
            Blackbox.log("SENSOR", "SENSOR FAILURE");
            telemetry.addData("Status", "SENSOR FAILURE");
            int failIndex = 1;
            for (String failure : failures) {
                telemetry.addData("Failure #" + (failIndex), failure);
                failIndex++;
            }
            telemetry.addLine(passedSensors + " other sensors passed");
            telemetry.update();
            while (true) {
                Robot.idle();
            }
        }
    }

    // Motor methods

    public static void leftMotors(double power) {
        leftMotor.setPower(power);
    }

    public static void rightMotors(double power) {
        rightMotor.setPower(power);
    }

    // Driving methods

    public static void turnToHeading(float targetHeading, double power) {
        float currentHeading = imu.getHeading();
        boolean turnLeft = (targetHeading - currentHeading > 0 ? true : false);
        while (true) {
            telemetry.addData("hdg", currentHeading);
            //telemetry.addData("phoneHdg", Robot.phoneGyro.getHeading());
            telemetry.update();

            double currentSpeed = power;
            float distanceTo = Math.abs(targetHeading - currentHeading);
            double minimumSpeed = 0.55f;
            double minimumLeftSpeed = (turnLeft ? -minimumSpeed : minimumSpeed);
            double minimumRightSpeed = (turnLeft ? minimumSpeed : -minimumSpeed);

            if (distanceTo < 10) {
                currentSpeed *= 0.20;
                currentSpeed = Math.min(currentSpeed, 0.6f);
            } else if (distanceTo < 20) {
                currentSpeed *= 0.30;
                currentSpeed = Math.min(currentSpeed, 0.65f);
            } else if (distanceTo < 30) {
                currentSpeed *= 0.40;
                currentSpeed = Math.min(currentSpeed, 0.7f);
            }

            leftMotors(Math.max(minimumLeftSpeed, (turnLeft ? -currentSpeed : currentSpeed)));
            rightMotors(Math.max(minimumRightSpeed, (turnLeft ? currentSpeed : -currentSpeed)));

            try { idle(); } catch (InterruptedException e) {}
            imu.update();
            currentHeading = imu.getHeading();

            turnLeft = (targetHeading - currentHeading > 0 ? true : false);
            /*if (turnLeft && targetHeading < currentHeading) {
                break;
            } else if (!turnLeft && targetHeading > currentHeading) {
                break;
            }*/
            if (targetHeading == currentHeading) {
                break;
            }
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }

    public static void turnToHeadingFast(float targetHeading, double power) {
        float currentHeading = imu.getHeading();
        boolean turnLeft = (targetHeading - currentHeading > 0 ? true : false);
        while (true) {
            telemetry.addData("hdg", currentHeading);
            //telemetry.addData("phoneHdg", Robot.phoneGyro.getHeading());
            telemetry.update();

            double currentSpeed = power;
            float distanceTo = Math.abs(targetHeading - currentHeading);
            double minimumSpeed = 0.55f;
            double minimumLeftSpeed = (turnLeft ? -minimumSpeed : minimumSpeed);
            double minimumRightSpeed = (turnLeft ? minimumSpeed : -minimumSpeed);

            leftMotors(Math.max(minimumLeftSpeed, (turnLeft ? -currentSpeed : currentSpeed)));
            rightMotors(Math.max(minimumRightSpeed, (turnLeft ? currentSpeed : -currentSpeed)));

            try { idle(); } catch (InterruptedException e) {}
            imu.update();
            currentHeading = imu.getHeading();

            turnLeft = (targetHeading - currentHeading > 0 ? true : false);
            /*if (turnLeft && targetHeading < currentHeading) {
                break;
            } else if (!turnLeft && targetHeading > currentHeading) {
                break;
            }*/
            if (Math.abs(targetHeading - currentHeading) < 10) {
                break;
            }
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }


    public static void moveForward_accel(double distanceToDrive, double power) throws InterruptedException {
        double distanceDriven = 0.0f;
        while (distanceDriven < (distanceToDrive * 100)) {
            leftMotors(power);
            rightMotors(power);
            imu.update();
            idle();
            distanceDriven += Math.abs(imu.getGravity().xAccel);
            telemetry.addData("distanceDriven", distanceDriven);
            telemetry.addData("encoder", leftMotor.getCurrentPosition());
            telemetry.update();
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }

    public static void moveForward_encoder(double distanceToDrive, double power) throws InterruptedException {
        double startPos = -leftMotor.getCurrentPosition();
        boolean negative = (distanceToDrive < 0);
        if (negative) {
            power *= -1;
        }
        while (
                (!negative && -leftMotor.getCurrentPosition() - startPos < distanceToDrive) ||
                (negative && -leftMotor.getCurrentPosition() - startPos > distanceToDrive)
        ) {
            double factor = 1.0f;
            if ((-leftMotor.getCurrentPosition() - startPos) > (distanceToDrive / 2)) {
                factor = 0.5f;
            }
            leftMotors(power * factor);
            rightMotors(power * factor);
            idle();
            telemetry.addData("curPos", -leftMotor.getCurrentPosition());
            telemetry.update();
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }

    public static void turnToHeadingWithPID(float targetHeading, double power) {
        float currentHeading = imu.getHeading();
        boolean turnLeft = (targetHeading - currentHeading > 0 ? true : false);

        double p;
        double i = 0;
        double d;
        double prevp = (targetHeading - currentHeading);

        double kp = .003;
        double ki = .0001;
        double kd = .003;

        double powerinput = power;

        while (true) {
            telemetry.addData("hdg", currentHeading);
            //telemetry.addData("phoneHdg", Robot.phoneGyro.getHeading());
            telemetry.update();

            double currentSpeed = power;
            float distanceTo = Math.abs(targetHeading - currentHeading);



            p = (targetHeading-currentHeading);
            i += p;
            d = p - prevp;

            prevp = d;

            powerinput = power + kp*p + ki*i + kd*d;

            telemetry.addData("p", p);
            telemetry.addData("k*p", kp*p);
            telemetry.addData("i", p);
            telemetry.addData("k*i", ki*i);
            telemetry.addData("d", d);
            telemetry.addData("k*d", kd*d);
            telemetry.addData("power", powerinput);


//            double minimumSpeed = 0.55f;
//            double minimumLeftSpeed = (turnLeft ? -minimumSpeed : minimumSpeed);
//            double minimumRightSpeed = (turnLeft ? minimumSpeed : -minimumSpeed);
//
//            if (distanceTo < 10) {
//                currentSpeed *= 0.20;
//                currentSpeed = Math.min(currentSpeed, 0.6f);
//            } else if (distanceTo < 20) {
//                currentSpeed *= 0.30;
//                currentSpeed = Math.min(currentSpeed, 0.65f);
//            } else if (distanceTo < 30) {
//                currentSpeed *= 0.40;
//                currentSpeed = Math.min(currentSpeed, 0.7f);
//            }
//
//            leftMotors(Math.max(minimumLeftSpeed, (turnLeft ? -currentSpeed : currentSpeed)));
//            rightMotors(Math.max(minimumRightSpeed, (turnLeft ? currentSpeed : -currentSpeed)));
            leftMotors(trim(powerinput));
            rightMotors(trim(powerinput));


            try { idle(); } catch (InterruptedException e) {}
            imu.update();
            currentHeading = imu.getHeading();

            turnLeft = (targetHeading - currentHeading > 0 ? true : false);
            /*if (turnLeft && targetHeading < currentHeading) {
                break;
            } else if (!turnLeft && targetHeading > currentHeading) {
                break;
            }*/
            if (targetHeading == currentHeading) {
                break;
            }
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }

    public static double trim (double number) {
        if (number > 1) {
            number = 1;
        } else if (number < -1) {
            number = -1;
        }
        return number;
    }

    // Sensing methods
    /*public static Alliance getBeaconRightColor() throws InterruptedException {
        beaconLeft.setPosition(0.0);
        beaconRight.setPosition(0.0);
        telemetry.addLine("WAITING FOR SERVO POS");
        telemetry.update();
        Thread.sleep(1000);
        Alliance response;
        if ((beaconColor.red() == beaconColor.blue()) || (beaconColor.red() == 0 && beaconColor.blue() == 255) || (beaconColor.blue() == 0 && beaconColor.red() == 255)) {
            response = Alliance.UNKNOWN;
        } else if (beaconColor.red() > beaconColor.blue()) {
            response = Alliance.RED;
        } else {
            response = Alliance.BLUE;
        }
        return response;
    }*/


}
