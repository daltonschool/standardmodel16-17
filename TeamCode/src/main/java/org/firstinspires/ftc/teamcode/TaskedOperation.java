package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.options.OptionManager;
import org.firstinspires.ftc.teamcode.tasks.AlignmentTaskIdk;
import org.firstinspires.ftc.teamcode.tasks.AlignmentTaskNew;
import org.firstinspires.ftc.teamcode.tasks.ButtonPressTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelEngageTask;
import org.firstinspires.ftc.teamcode.tasks.MoveForwardFastInaccurateTask;
import org.firstinspires.ftc.teamcode.tasks.MoveForwardTask;
import org.firstinspires.ftc.teamcode.tasks.MoveUntilLineTask;
import org.firstinspires.ftc.teamcode.tasks.ShootTask;
import org.firstinspires.ftc.teamcode.tasks.SpookyTestTask;
import org.firstinspires.ftc.teamcode.tasks.TurnToHeadingTask;
import org.firstinspires.ftc.teamcode.tasks.TurnUntilLineTask;
import org.firstinspires.ftc.teamcode.taskutil.Task;

import java.util.ArrayList;

public abstract class TaskedOperation extends LinearOpMode {
    public abstract Alliance getCurrentAlliance();
    public abstract boolean isASpookster();

    @Override
    public void runOpMode() throws InterruptedException {
        Blackbox.init();                                // set up logging

        Blackbox.log("INFO", "Current alliance: " + (getCurrentAlliance() == Alliance.RED ? "red" : "blue"));

        telemetry.addData("Status", "Starting robot...");
        telemetry.update();

        Robot.init(this);                               // set up robot

        telemetry.addData("Status", "Starting options...");
        telemetry.update();

        OptionManager.init();                           // set up options
        Robot.currentAlliance = getCurrentAlliance();   // set up alliance
        Robot.verifyAllSensors();                       // check all sensors
        OptionManager.printAllOptions();                // print out options for verification

        int blueNegativeFactor = (Robot.currentAlliance == Alliance.BLUE ? -1 : 1);

        boolean shooting = true;
        boolean getBeacons = true;
        boolean getCapBall = false;

        // set up tasks
        ArrayList<Task> tasks = new ArrayList<Task>();
        tasks.add(new FlywheelEngageTask(null));
        tasks.add(new MoveForwardTask(2100));
        if (shooting) {
            tasks.add(new ShootTask(null));
        }

        if (getBeacons) {
            // first beacon
            tasks.add(new TurnToHeadingTask(67 * blueNegativeFactor));
            tasks.add(new MoveForwardTask(1800));
            tasks.add(new MoveUntilLineTask(null));
            tasks.add(new TurnUntilLineTask(null));
            tasks.add(new AlignmentTaskIdk((Robot.currentAlliance == Alliance.RED ? Robot.vuforia.gears : Robot.vuforia.wheels)));
            tasks.add(new ButtonPressTask(null));

            // go to second beacon
            tasks.add(new TurnToHeadingTask(0 * blueNegativeFactor));
            tasks.add(new MoveForwardFastInaccurateTask(2200));
            tasks.add(new MoveUntilLineTask(null));
            tasks.add(new MoveForwardTask(220));
            tasks.add(new TurnUntilLineTask(null));
            tasks.add(new TurnToHeadingTask(90 * blueNegativeFactor));
            tasks.add(new AlignmentTaskIdk((Robot.currentAlliance == Alliance.RED ? Robot.vuforia.tools : Robot.vuforia.legos)));
            tasks.add(new TurnToHeadingTask(90 * blueNegativeFactor));
            tasks.add(new ButtonPressTask(null));
        }
/*
        if (getCapBall) {
            tasks.add(new TurnToHeadingTask(110 * blueNegativeFactor));
            tasks.add(new MoveForwardTask(-6500));
        }*/

        if (isASpookster()) {
            Blackbox.log("INFO", "we have a spookster!!!");
            tasks.clear();
            //tasks.add(new SpookyTestTask(null));
            //tasks.add(new TurnUntilLineTask(null));
            tasks.add(new AlignmentTaskIdk(Robot.vuforia.gears));
        }

        // init tasks
        for (Task t : tasks) {
            t.init();
        }

        // we're ready
        Blackbox.log("INFO", "READY TO GO!");
        telemetry.addData("Status", "READY TO GO!");
        telemetry.update();

        waitForStart();

        Robot.start();
        Robot.beaconLeft.setPosition(0.0);
        Robot.beaconRight.setPosition(0.0);

        if (!isASpookster()) { Robot.nomMiddle.setPower(1.0f); }
        while (opModeIsActive()) {
            int taskIndex = 0;
            for (Task t : tasks) {
                Blackbox.log("TASK", "Current task: " + (taskIndex + 1));
                telemetry.addData("Current task: ", taskIndex + 1);

                CameraDevice.getInstance().setFlashTorchMode(false);
                t.run();
                CameraDevice.getInstance().setFlashTorchMode(false);

                Robot.update();
                Robot.idle();

                taskIndex++;
            }

            Blackbox.log("INFO", "All tasks completed.");
            telemetry.addData("Status", "All tasks completed.");
            telemetry.update();

            requestOpModeStop();
            return;
        }

    }
}
