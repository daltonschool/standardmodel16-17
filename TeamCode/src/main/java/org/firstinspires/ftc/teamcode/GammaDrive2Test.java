/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "GammaDrive2Test", group = "TeleOp")
public class GammaDrive2Test extends OpMode {


    DcMotor launchLeft;
    DcMotor launchRight;

    DcMotor leftback;
    DcMotor rightback;

    DcMotor inside_nom;
    DcMotor outside_nom;
    DcMotor lift;

    double powerleft;
    double powerright;

    double launchpower;

    double launchVoltage;

    double prevtime;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //
        launchLeft = hardwareMap.dcMotor.get("launch_left");
        launchRight = hardwareMap.dcMotor.get("launch_right");

        leftback = hardwareMap.dcMotor.get("drive_left");
        rightback = hardwareMap.dcMotor.get("drive_right");
        inside_nom = hardwareMap.dcMotor.get("inside_nom");
        outside_nom = hardwareMap.dcMotor.get("outside_nom");
        lift = hardwareMap.dcMotor.get("lift");


        launchVoltage = 7;

        prevtime = 0;


    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());


        if (runtime.time() - prevtime >= 5) {
            launchpower = launchVoltage/hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
            prevtime = runtime.time();
        }

        if (gamepad2.dpad_up == true && Math.abs(launchpower) < 1) {
            launchVoltage += 1;
        }
        if (gamepad2.dpad_down ==  true && Math.abs(launchpower) < 1) {
            launchVoltage -= 1;
        }
//
//        //Launch
//        if (gamepad1.a == true) {
//            launchRight.setPower(launchpower);
//            launchLeft.setPower(-launchpower);
//        } else if (gamepad1.b == true) {
//            launchRight.setPower(-.4);
//            launchLeft.setPower(.4);
//        } else if (gamepad1.x == true) {
//            launchRight.setPower(-.6);
//            launchLeft.setPower(.6);
//        } else if (gamepad1.y == true) {
//            launchRight.setPower(.3);
//            launchLeft.setPower(-.3);
//        } else {
//            launchRight.setPower(0.0);
//            launchLeft.setPower(0.0);
//        }

        telemetry.addData("Voltage:", hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage());

        telemetry.addData("Launch Power:", launchpower);

        telemetry.addData("Launch Voltage Target", launchVoltage);
        telemetry.addData("Launch Voltage Actual", launchpower*hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage());

        if (gamepad1.a == true) {
            launchRight.setPower(-launchpower);
            launchLeft.setPower(launchpower);
        } else {
            launchRight.setPower(0.0);
            launchLeft.setPower(0.0);
        }

        //Drive

        powerright = gamepad1.left_stick_y;
        powerleft = gamepad1.left_stick_y;

        powerright += gamepad1.right_stick_x;
        powerleft -= gamepad1.right_stick_x;

        leftback.setPower(-trim(powerleft));
        rightback.setPower(trim(powerright));


        if (gamepad2.a == true) {
            inside_nom.setPower(1);
        } else if (gamepad2.y == true) {
            inside_nom.setPower(-1);
        } else {
            inside_nom.setPower(0);
        }

        if (gamepad2.b == true) {
            outside_nom.setPower(1);
        } else if (gamepad2.x == true) {
            outside_nom.setPower(-1);
        } else {
            outside_nom.setPower(0);
        }

        //Lift
        if (gamepad2.right_trigger != 0 && gamepad2.left_trigger == 0 && gamepad2.left_bumper == false && gamepad2.right_bumper == false) {
            lift.setPower(-.5);
        } else if (gamepad2.left_trigger != 0 && gamepad2.right_trigger == 0 && gamepad2.left_bumper == false && gamepad2.right_bumper == false) {
            lift.setPower(-5);
        } else if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_bumper == true && gamepad2.right_bumper == false) {
            lift.setPower(1);
        } else if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_bumper == false && gamepad2.right_bumper == true) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        //

//        if (gamepad1.dpad_down == true) {
//            nom.setPower(.5);
//        }
//        if (gamepad1.dpad_right == true) {
//            nom.setPower(.5);
////            feeder.setPower(.3);
//        }
//        if (gamepad1.dpad_up == true) {
////            feeder.setPower(.3);
//        }
    }

    public double trim (double number) {
        if (number > 1) {
            number = 1;
        } else if (number < -1) {
            number = -1;
        }
        return number;
    }
}