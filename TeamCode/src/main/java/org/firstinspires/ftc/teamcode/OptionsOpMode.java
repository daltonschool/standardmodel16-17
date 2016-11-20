package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

@Autonomous(name="Options")
public class OptionsOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        try {
            OptionsHTTPD o = new OptionsHTTPD();
        } catch (IOException e) {
            telemetry.addLine("Error loading optionshttpd!");
            telemetry.update();
            e.printStackTrace();
            requestOpModeStop();
            return;
        }
        while (opModeIsActive()) {
            telemetry.addLine("Listening on port 9372!");
            telemetry.update();
            idle();
        }
    }
}