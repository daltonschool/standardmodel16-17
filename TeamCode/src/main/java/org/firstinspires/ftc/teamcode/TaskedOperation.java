package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.options.OptionManager;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;
import org.firstinspires.ftc.teamcode.tasks.AlignmentTask;
import org.firstinspires.ftc.teamcode.tasks.ButtonPressTask;
import org.firstinspires.ftc.teamcode.tasks.LineFollowingTask;
import org.firstinspires.ftc.teamcode.tasks.MoveForwardTask;
import org.firstinspires.ftc.teamcode.tasks.MoveUntilLineTask;
import org.firstinspires.ftc.teamcode.tasks.ShootTask;
import org.firstinspires.ftc.teamcode.tasks.TurnToHeadingTask;
import org.firstinspires.ftc.teamcode.tasks.TurnUntilLineTask;
import org.firstinspires.ftc.teamcode.tasks.VuforiaAlignmentTask;
import org.firstinspires.ftc.teamcode.taskutil.Task;

import java.lang.reflect.Field;
import java.util.ArrayList;

public abstract class TaskedOperation extends LinearOpMode {
    public abstract Alliance getCurrentAlliance();
    public abstract boolean isASpookster();

    @Override
    public void runOpMode() throws InterruptedException {

        // set up logging
        Blackbox.init();
        Blackbox.log("INFO", "Current alliance: " + (getCurrentAlliance() == Alliance.RED ? "red" : "blue"));

        // set up robot
        Robot.init(this);

        // set up options
        OptionManager.init();

        // set up alliance
        Robot.currentAlliance = getCurrentAlliance();

        // print out options for verification
        Field[] fields = OptionManager.getOptionFields();
        for (Field f : fields) {
            try {
                telemetry.addData(OptionManager.getPrettyName(f.getName()), f.get(OptionManager.currentOptions));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
                telemetry.addData(OptionManager.getPrettyName(f.getName()), "IllegalAccessException");
            }
        }

        int blueNegativeFactor = (Robot.currentAlliance == Alliance.BLUE ? -1 : 1);
        int firstTurn = (Robot.currentAlliance == Alliance.RED ? 48 : 42);

        boolean shooting = true;
        boolean getBeacons = true;
        boolean getCapBall = false;

        // set up tasks
        ArrayList<Task> tasks = new ArrayList<Task>();
        //tasks.add(new LineFollowingTask(null));
        tasks.add(new MoveForwardTask(2300));
        if (shooting) {
            tasks.add(new ShootTask(null));
        }

        if (getBeacons) {
            // first beacon
            tasks.add(new TurnToHeadingTask(firstTurn * blueNegativeFactor));
            tasks.add(new MoveForwardTask(1400));
            tasks.add(new MoveUntilLineTask(null));
            tasks.add(new TurnToHeadingTask(80 * blueNegativeFactor));
            tasks.add(new AlignmentTask((Robot.currentAlliance == Alliance.RED ? Robot.vuforia.gears : Robot.vuforia.wheels)));
            //tasks.add(new MoveForwardTask(-200));
            tasks.add(new ButtonPressTask(null));

            // go to second beacon
            tasks.add(new TurnToHeadingTask(20 * blueNegativeFactor));
            tasks.add(new MoveForwardTask(2200));
            tasks.add(new MoveUntilLineTask(null));
            tasks.add(new MoveForwardTask(250));
            tasks.add(new TurnUntilLineTask(null));
            tasks.add(new AlignmentTask((Robot.currentAlliance == Alliance.RED ? Robot.vuforia.tools : Robot.vuforia.legos)));
            tasks.add(new ButtonPressTask(null));
        }
/*
        if (getCapBall) {
            tasks.add(new TurnToHeadingTask(110 * blueNegativeFactor));
            tasks.add(new MoveForwardTask(-6500));
        }*/

        if (isASpookster()) {
            Blackbox.log("INFO", "we have a spookster!!!");
            Blackbox.log("INFO", "we have a spookster!!!");
            Blackbox.log("INFO", "we have a spookster!!!");
            tasks.clear();
            tasks.add(new AlignmentTask(Robot.vuforia.wheels));
        }

        // init tasks
        for (Task t : tasks) {
            t.init();
        }

        // calibrate
        Blackbox.log("INFO", "Calibrating for black level...");
        telemetry.addData("Status", "Calibrating for black level...");
        telemetry.update();

        Robot.leftLineColor.blackLevelCalibration();
        Robot.rightLineColor.blackLevelCalibration();
        Thread.sleep(2000);


        // we're ready
        Blackbox.log("INFO", "READY TO GO!");
        telemetry.addData("Status", "READY TO GO!");
        telemetry.update();

        waitForStart();

        Robot.start();
        Robot.beaconLeft.setPosition(0.0);
        Robot.beaconRight.setPosition(0.0);

        Robot.nomMiddle.setPower(1.0f);

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
