package org.firstinspires.ftc.teamcode.tasks;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.ColorSensors;
import org.firstinspires.ftc.teamcode.taskutil.Task;
import org.firstinspires.ftc.teamcode.Robot;
import static org.firstinspires.ftc.teamcode.Robot.leftMotors;
import static org.firstinspires.ftc.teamcode.Robot.rightMotors;

/**
 * Created by student on 11/21/16.
 */
public class LineFollowingTask extends Task {
    public LineFollowingTask(Object e) {
        super(e);
    }
    ColorSensors obj1 = new ColorSensors();

    @Override
    public void init() {

    }

    @Override
    public void run()  {
        //to set correct right side color
        Alliance rightColor;
        if (Robot.beaconColor.red() == Robot.beaconColor.blue()) {
            rightColor = Alliance.UNKNOWN;
        } else if (Robot.beaconColor.red() > Robot.beaconColor.blue()) {
            rightColor = Alliance.RED;
        } else {
            rightColor = Alliance.BLUE;
        }

        //actual line following code
        while (rightColor == Alliance.UNKNOWN) {
            //gets values of the 2 sensors
            double sensorVal_1 = obj1.getColorSensorVal("ls1");
            double sensorVal_2 = obj1.getColorSensorVal("ls2");

            //is values of the mat and line
            double whiteLineIntensityValue = 1.0; //REPLACE WITH CORRECT VALUE
            double blackMatIntensityValue = 0.0;  //ALSO REPLACE HERE

            //threshold of when the sensor is reading while and when it is reading black
            double thresholdVal = (whiteLineIntensityValue + blackMatIntensityValue)/2; //REPLACE WITH DERIVED VALUE

            //simple logic for line following
            if(sensorVal_1 < thresholdVal && sensorVal_2 < thresholdVal) { //if on course (black and black)
                rightMotors(.8);
                leftMotors(.8);
            }
            else if(sensorVal_1 > thresholdVal && sensorVal_2 < thresholdVal){ //if needs to turn left (white and black)
                rightMotors(.8);
                leftMotors(-.8);
            }
            else if(sensorVal_1 < thresholdVal && sensorVal_2 > thresholdVal){ //if needs to turn right
                rightMotors(-.8);
                leftMotors(.8);
            }
            else {
                Robot.telemetry.addData("Error", "I am lost");
                Robot.telemetry.update();
            }
        }
        rightMotors(0);
        leftMotors(0);
    }
}
