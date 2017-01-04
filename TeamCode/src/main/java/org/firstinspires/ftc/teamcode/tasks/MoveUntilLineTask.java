package org.firstinspires.ftc.teamcode.tasks;

import android.util.Log;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.MRColorSensor;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class MoveUntilLineTask extends Task {
    public MoveUntilLineTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        Robot.leftMotors(0.18f);
        Robot.rightMotors(0.18f);

        MRColorSensor checkingColorSensor = Robot.leftLineColor;
        if (Robot.currentAlliance == Alliance.RED) {
            checkingColorSensor = Robot.leftLineColor;
        } else if (Robot.currentAlliance == Alliance.BLUE) {
            checkingColorSensor = Robot.rightLineColor;
        }

        //byte redReading = checkingColorSensor.redReading();
        //byte blueReading = checkingColorSensor.blueReading();
        int whiteReading = ((Byte)checkingColorSensor.whiteReading()).intValue() - (int)6;
        while ((whiteReading < 5)) {// || (blueReading > 10 && redReading < 10) || (redReading > 10 && blueReading < 10)) {
            //Robot.telemetry.addData("leftRedReading", redReading);
            //Robot.telemetry.addData("leftBlueReading", blueReading);
            Robot.telemetry.addData("leftWhiteReading", whiteReading);
            /*Log.i("leftRedReading", Byte.toString(redReading));
            Log.i("leftBlueReading", Byte.toString(blueReading));
            Log.i("leftWhiteReading", Byte.toString(whiteReading));*/
            Robot.telemetry.update();
            Robot.update();
            Robot.idle();

            //redReading = checkingColorSensor.redReading();
            //blueReading = checkingColorSensor.blueReading();
            whiteReading = (int)checkingColorSensor.whiteReading() - (int)6;
        }

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}
