package org.firstinspires.ftc.teamcode.tasks;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import static org.firstinspires.ftc.teamcode.Robot.leftMotors;
import static org.firstinspires.ftc.teamcode.Robot.rightMotors;

/**
 * Created by student on 11/21/16.
 */
public class LineFollowingTask extends OpMode {

    public static ColorSensor lineSensor_1 = null;
    public static ColorSensor lineSensor_2 = null;
    double lastError = 0;
    double rightBaseSpeed = .5;
    double leftBaseSpeed = .5;
    boolean nearBeacon = false; //replace with condition to check we are close to the beacon
//    double rightMaxSpeed = 1;
//    double leftMaxSpeed = 1;
//    double Kp = 0;
//    double Kd = 0;

    @Override
    public void init() {
        lineSensor_1 = hardwareMap.colorSensor.get("ls 1");
        lineSensor_2 = hardwareMap.colorSensor.get("ls 2");
    }

    @Override
    public void loop()  {
        while (!nearBeacon) {
            float hsvValues_1[] = {0F,0F,0F};
            float hsvValues_2[] = {0F,0F,0F};
            Color.RGBToHSV((lineSensor_1.red() * 255) / 800, (lineSensor_1.green() * 255) / 800, (lineSensor_1.blue() * 255) / 800, hsvValues_1);
            double sensorVal_1 = hsvValues_1[1];
            double sensorVal_2 = hsvValues_2[1];
            double whiteLineIntensityValue = 1; //Replace with value that we get
            double blackMatIntensityValue = 0;  //Also do that here
           // int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
            //double error = colorVal - targetIntensity;

            //double motorSpeed = Kp * error + Kd * (error - lastError);
            //lastError = error;
            if(sensorVal_1 > .5 && sensorVal_2 > .5) { //if on course
                rightMotors(.8);
                leftMotors(.8);
            }
            else if(sensorVal_1<.5 && sensorVal_2>.5){ //if needs to turn left
                rightMotors(.8);
                leftMotors(-.8);
            }
            else if(sensorVal_1>.5 && sensorVal_2<.5){ //if needs to turn left
                rightMotors(-.8);
                leftMotors(.8);
            }
            else {
                telemetry.addData("Error", "I am lost");
            }
//            if (rightMotorSpeed > rightMaxSpeed ) {
//                rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
//            }
//            if (leftMotorSpeed > leftMaxSpeed ) {
//                leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
//            }
//
//            rightMotors(rightMotorSpeed);
//            leftMotors(leftMotorSpeed);
            if () { //close to beacon
                nearBeacon = true;
            }
        }
        rightMotors(0);
        leftMotors(0);
    }
}
