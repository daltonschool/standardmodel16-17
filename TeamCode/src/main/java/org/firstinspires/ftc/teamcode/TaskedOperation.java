package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.options.OptionManager;
import org.firstinspires.ftc.teamcode.tasks.AlignmentTaskIdk;
import org.firstinspires.ftc.teamcode.tasks.AlignmentTaskNew;
import org.firstinspires.ftc.teamcode.tasks.AttackCapBallTask;
import org.firstinspires.ftc.teamcode.tasks.ButtonPressTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelEngageTask;
import org.firstinspires.ftc.teamcode.tasks.LineTestTask;
import org.firstinspires.ftc.teamcode.tasks.MoveBackUntilFrontLineTask;
import org.firstinspires.ftc.teamcode.tasks.MoveForwardFastInaccurateTask;
import org.firstinspires.ftc.teamcode.tasks.MoveForwardTask;
import org.firstinspires.ftc.teamcode.tasks.MoveUntilLineTask;
//import org.firstinspires.ftc.teamcode.tasks.ShootTask;
import org.firstinspires.ftc.teamcode.tasks.ShootTask;
import org.firstinspires.ftc.teamcode.tasks.SpookyTestTask;
import org.firstinspires.ftc.teamcode.tasks.TurnTestTask;
import org.firstinspires.ftc.teamcode.tasks.TurnToHeadingTask;
import org.firstinspires.ftc.teamcode.tasks.TurnToSecondBeaconTask;
import org.firstinspires.ftc.teamcode.tasks.TurnUntilLineTask;
import org.firstinspires.ftc.teamcode.tasks.WaitTask;
import org.firstinspires.ftc.teamcode.taskutil.Task;

import java.util.ArrayList;

public abstract class TaskedOperation extends LinearOpMode {
    public abstract Alliance getCurrentAlliance();
    public abstract boolean isASpookster();
    public abstract boolean isShotsOnly();

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

        double voltage = Robot.voltageSensor.getVoltage();

        boolean shooting = true;
        boolean getBeacons = true;
        boolean getCapBall = false;

        // set up tasks
        ArrayList<Task> tasks = new ArrayList<Task>();

        //tasks.add(new FlywheelEngageTask(null));
        tasks.add(new MoveForwardTask(1100));

        /*if (shooting) {
            tasks.add(new ShootTask(null));
        }*/

        if (getBeacons) {
            // first beacon
            tasks.add(new TurnToHeadingTask((Robot.currentAlliance == Alliance.RED ? 60 : -60)));
            tasks.add(new MoveForwardFastInaccurateTask(1200));
            tasks.add(new MoveUntilLineTask(null));
            tasks.add(new MoveForwardTask(-200));
            tasks.add(new TurnUntilLineTask(null));
            tasks.add(new AlignmentTaskIdk((Robot.currentAlliance == Alliance.RED ? Robot.vuforia.gears : Robot.vuforia.wheels)));
            tasks.add(new TurnToHeadingTask(90 * blueNegativeFactor));
            tasks.add(new MoveForwardTask(-100));
            tasks.add(new ButtonPressTask(null));
            tasks.add(new MoveForwardTask(-150));

            // go to second beacon
            tasks.add(new TurnToHeadingTask(-4));
            tasks.add(new MoveForwardFastInaccurateTask(1750));
            tasks.add(new MoveUntilLineTask(null));
            tasks.add(new MoveBackUntilFrontLineTask(null));
            if (voltage < 12.5) {
                tasks.add(new MoveForwardTask(-100));
            } else {
                tasks.add(new MoveForwardTask(-5));
            }
            tasks.add(new TurnToHeadingTask(90 * blueNegativeFactor));
            tasks.add(new AlignmentTaskIdk((Robot.currentAlliance == Alliance.RED ? Robot.vuforia.tools : Robot.vuforia.legos)));
            tasks.add(new WaitTask(100));
            tasks.add(new ButtonPressTask(null));

//            tasks.add(new MoveBackUntilFrontLineTask(null));
//            tasks.add(new MoveForwardTask(-50));
//            tasks.add(new TurnToHeadingTask(90 * blueNegativeFactor));
//            tasks.add(new WaitTask(100));
//            tasks.add(new ButtonPressTask(null));
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

            tasks.add(new MoveUntilLineTask(null));
            tasks.add(new MoveBackUntilFrontLineTask(null));
            if (voltage < 12.5) {
                tasks.add(new MoveForwardTask(-100));
            } else {
                tasks.add(new MoveForwardTask(-5));
            }
            tasks.add(new TurnToHeadingTask(90 * blueNegativeFactor));
            tasks.add(new AlignmentTaskIdk((Robot.currentAlliance == Alliance.RED ? Robot.vuforia.tools : Robot.vuforia.legos)));
            tasks.add(new WaitTask(100));
            tasks.add(new ButtonPressTask(null));
            /*tasks.add(new MoveForwardTask(-200));
            tasks.add(new TurnUntilLineTask(null));
            tasks.add(new AlignmentTaskIdk((Robot.currentAlliance == Alliance.RED ? Robot.vuforia.tools : Robot.vuforia.legos)));
            tasks.add(new TurnToHeadingTask(90 * blueNegativeFactor));*/
        }

        if (isShotsOnly()) {
            Blackbox.log("INFO", "Shots only!");
            tasks.clear();

            tasks.add(new FlywheelEngageTask(null));
            tasks.add(new MoveForwardTask(1100));
            tasks.add(new ShootTask(null));
            tasks.add(new MoveForwardTask(1500));
            tasks.add(new AttackCapBallTask(null));
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

        telemetry.addData("Status", "ihuaefguyadfsuhifsjkhafskjlhadfs");
        telemetry.update();
        /*requestOpModeStop();
        if (true) {
            return;
        }*/

        Robot.start();
        Robot.extendBoth();

        //if (!isASpookster()) { Robot.nom.setPower(1.0f); }

        int taskIndex = 0;
        for (Task t : tasks) {
            Blackbox.log("TASK", "Current task: " + (taskIndex + 1));
            telemetry.addData("Current task: ", taskIndex + 1);

            // CameraDevice.getInstance().setFlashTorchMode(false);
            t.run();
            // CameraDevice.getInstance().setFlashTorchMode(false);

            Robot.update();
            Robot.idle();

            taskIndex++;
        }

        Blackbox.log("INFO", "All tasks completed.");
        telemetry.addData("Status", "All tasks completed.");
        telemetry.update();

        requestOpModeStop();

    }
}