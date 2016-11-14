package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    // Motors
    public static DcMotor leftMotor;
    public static DcMotor rightMotor;

    public static DcMotor flywheelLeft;
    public static DcMotor flywheelRight;

    public static DcMotor nomFront;
    public static DcMotor nomMiddle;

    public static DcMotor conveyor;

    // Servos
    public static Servo beaconLeft;
    public static Servo beaconRight;

    // Sensors
    public static ColorSensor colorSensor = null;
    public static ColorSensor beaconColor = null;

    public static IMU imu = null;

    public static Vuforia vuforia;

    // Telemetry
    public static Telemetry telemetry;

    // The original opmode.
    public static LinearOpMode opMode;

    public static void init(LinearOpMode om) {
        opMode = om;
        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        // Motors
        leftMotor = hardwareMap.dcMotor.get("left motor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        rightMotor = hardwareMap.dcMotor.get("right motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        flywheelLeft = hardwareMap.dcMotor.get("flywheel left");
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);

        flywheelRight = hardwareMap.dcMotor.get("flywheel right");
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        nomMiddle = hardwareMap.dcMotor.get("nom middle");
        nomMiddle.setDirection(DcMotor.Direction.FORWARD);

        nomFront = hardwareMap.dcMotor.get("nom front");
        nomFront.setDirection(DcMotor.Direction.FORWARD);

        conveyor = hardwareMap.dcMotor.get("conveyor");
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        // Servos
        beaconLeft = hardwareMap.servo.get("beacon left");
        beaconRight = hardwareMap.servo.get("beacon right");

        // Sensors
        // Color
        //colorSensor = hardwareMap.colorSensor.get("color sensor");
        beaconColor = hardwareMap.colorSensor.get("beacon color");
        beaconColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        beaconColor.enableLed(false);

        // IMU
        imu = new IMU();
        imu.init(hardwareMap.get(BNO055IMU.class, "imu"));

        // Vuforia
        vuforia = new Vuforia();
        vuforia.init();
    }

    public static void start() {
        vuforia.start();
    }

    public static void update() {
        // update sensors
        imu.update();
        vuforia.update();

        // and telemetry
        telemetry.update();
    }

    public static void idle() throws InterruptedException {
        opMode.idle();
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
            telemetry.update();

            double currentSpeed = power;
            float distanceTo = Math.abs(targetHeading - currentHeading);

            if (distanceTo < 10) {
                currentSpeed *= 0.30;
            } else if (distanceTo < 20) {
                currentSpeed *= 0.40;
            } else if (distanceTo < 30) {
                currentSpeed *= 0.60;
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
        double startPos = leftMotor.getCurrentPosition();
        while ((leftMotor.getCurrentPosition() - startPos) < distanceToDrive) {
            double factor = 1.0f;
            if ((leftMotor.getCurrentPosition() - startPos) > (distanceToDrive / 2)) {
                factor = 0.5f;
            }
            leftMotors(power * factor);
            rightMotors(power * factor);
            idle();
            telemetry.addData("curPos", leftMotor.getCurrentPosition());
            telemetry.update();
        }
        leftMotors(0.0);
        rightMotors(0.0);
    }

    // Sensing methods
    public static Alliance getBeaconRightColor() throws InterruptedException {
        beaconLeft.setPosition(0.0);
        beaconRight.setPosition(0.0);
        telemetry.addLine("WAITING FOR SERVO POS");
        telemetry.update();
        Thread.sleep(1000);
        Alliance response;
        if (beaconColor.red() != 0 && beaconColor.red() != 255 && beaconColor.red() > 30) {
            response = Alliance.RED;
        } else {
            response = Alliance.BLUE;
        }
        return response;
    }

    // Other
    /*public void lineFollower() throws InterruptedException {

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
    }*/
}
