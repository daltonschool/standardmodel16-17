package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.options.OptionManager;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;
import org.firstinspires.ftc.teamcode.tasks.AlignmentTask;
import org.firstinspires.ftc.teamcode.taskutil.Task;

import java.util.ArrayList;

public abstract class TaskedOperation extends LinearOpMode {
    public abstract Alliance getCurrentAlliance();

    @Override
    public void runOpMode() throws InterruptedException {

        // set up logging
        Blackbox.init();
        Blackbox.log("INFO", "Current alliance: " + (getCurrentAlliance() == Alliance.RED ? "red" : "blue"));

        // set up robot
        Robot.init(this);

        // set up options
        OptionManager.init();

        // TODO: print out options for verification

        // set up tasks
        ArrayList<Task> tasks = new ArrayList<Task>();
        tasks.add(new AlignmentTask(Robot.vuforia.gears));

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

        while (opModeIsActive()) {
            int taskIndex = 0;
            for (Task t : tasks) {
                Blackbox.log("TASK", "Current task: " + (taskIndex + 1));
                telemetry.addData("Current task: ", taskIndex + 1);

                t.run();

                Robot.update();
                Robot.idle();

                taskIndex++;
            }

            Blackbox.log("INFO", "All tasks completed.");
            telemetry.addData("Status", "All tasks completed.");
            telemetry.update();
        }
    }
}
