package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

import java.util.ArrayList;

public class TurnToSecondBeaconTask extends Task {
    public TurnToSecondBeaconTask(Object e) {
        super(e);
    }

    @Override
    public void init() {
    }

    @Override
    public void run() throws InterruptedException {
        while (true) {
            ArrayList<Task> task = new ArrayList<Task>();
            int blueNegativeFactor = (Robot.currentAlliance == Alliance.BLUE ? -1 : 1);

            Robot.update();

//            double leftLine = Robot.leftLineLight.getLightDetected();
//            double rightLine = Robot.rightLineLight.getLightDetected();
            Robot.telemetry.addData("Left line light", Robot.leftLineLight.getLightDetected());
            Robot.telemetry.addData("Right line light", Robot.rightLineLight.getLightDetected());
            Robot.telemetry.update();

            if (Robot.leftLineLight.getLightDetected() > Robot.ODS_BLACK_VALUE || Robot.rightLineLight.getLightDetected() > Robot.ODS_BLACK_VALUE) {
                Robot.telemetry.addData("On The Line", 1);
                Robot.telemetry.update();
                Task turn1 = new TurnToHeadingTask(90 * blueNegativeFactor);
                turn1.init();
                turn1.run();
                if (Robot.leftLineLight.getLightDetected() > Robot.ODS_BLACK_VALUE || Robot.rightLineLight.getLightDetected() > Robot.ODS_BLACK_VALUE) {
                    Robot.telemetry.addData("On The Line", 2);
                    Robot.telemetry.update();
                    task.add(new AlignmentTaskIdk(null));
                    task.add(new TurnToHeadingTask(90 * blueNegativeFactor));
                    task.add(new MoveForwardTask(-100));
                    task.add(new ButtonPressTask(null));
                } else {
                    Robot.telemetry.addData("Off The Line", 2);
                    Robot.telemetry.update();
                    task.add(new MoveToWallTask(null));
                    task.add(new TurnToHeadingTask(90 * blueNegativeFactor));
                    task.add(new MoveForwardTask(-100));
                    task.add(new ButtonPressTask(null));
                }
            } else {
                Robot.telemetry.addData("Off The Line", 1);
                Robot.telemetry.update();
                task.add(new TurnUntilLineTask(null));
                task.add(new AlignmentTaskIdk(null));
                task.add(new TurnToHeadingTask(90 * blueNegativeFactor));
                task.add(new MoveForwardTask(-100));
                task.add(new ButtonPressTask(null));
            }

            for (Task t : task) {
                t.init();
            }

            for (Task t : task) {
                t.run();
            }

            Robot.idle();
        }
    }
}

