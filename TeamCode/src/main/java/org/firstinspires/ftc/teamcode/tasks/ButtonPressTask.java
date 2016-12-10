package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Blackbox;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class ButtonPressTask extends Task {
    public ButtonPressTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        Robot.beaconLeft.setPosition(0.0);
        Robot.beaconRight.setPosition(0.0);
        Robot.telemetry.addLine("WAITING FOR SERVO POS");
        Robot.telemetry.update();
        Thread.sleep(500);

        // find the right color
        Alliance rightColor;
        if (Robot.beaconColor.red() == Robot.beaconColor.blue()) {
            rightColor = Alliance.UNKNOWN;
        } else if (Robot.beaconColor.red() > Robot.beaconColor.blue()) {
            rightColor = Alliance.RED;
        } else {
            rightColor = Alliance.BLUE;
        }

        // check if we found it
        if (rightColor == Alliance.UNKNOWN) {
            // ahh!
            Blackbox.log("INFO", "Could not find color!");
            Robot.telemetry.addLine("Could not find color!");
            Robot.telemetry.update();

            // back up
            Robot.leftMotors(-0.4f);
            Robot.rightMotors(-0.4f);
            Thread.sleep(500);
            Robot.leftMotors(0.0f);
            Robot.rightMotors(0.0f);

            return;
        }

        // extend correct arm
        if (rightColor == Robot.currentAlliance) {
            Blackbox.log("INFO", "Pressing RIGHT");
            Robot.telemetry.addLine("PRESSING RIGHT");
            Robot.beaconLeft.setPosition(0.0);
            Robot.beaconRight.setPosition(0.0);
        } else {
            Blackbox.log("INFO", "Pressing LEFT");
            Robot.telemetry.addLine("PRESSING LEFT");
            Robot.beaconLeft.setPosition(1.0);
            Robot.beaconRight.setPosition(1.0);
        }
        Robot.telemetry.update();
        Thread.sleep(500);

        // move forwards
        Robot.leftMotors(0.5f);
        Robot.rightMotors(0.5f);
        Thread.sleep(1500);
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);

        // retreat
        Robot.leftMotors(-0.5f);
        Robot.rightMotors(-0.5f);
        Thread.sleep(350);
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);

        // reset
        Robot.beaconLeft.setPosition(0.0);
        Robot.beaconRight.setPosition(0.0);
    }
}
