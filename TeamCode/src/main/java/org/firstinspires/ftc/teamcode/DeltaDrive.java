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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "DeltaDrive", group = "TeleOp")
public class DeltaDrive extends OpMode {


    DcMotor launchLeft;
    DcMotor launchRight;

    DcMotor leftback;
    DcMotor rightback;

    DcMotor nom;
    DcMotor lift;

    DcMotor cbleft;
    DcMotor cbright;
    double cbpowerMultiplier = 1;
    double cbmaxLiftHeight = 3200;
    double cbstoppingDistance = 100;
    double cbstartPosL;
    double cbstartPosR;
    int cbleftState = 0;
    int cbrightState = 0;

    Servo bpleft;
    Servo bpright;

    double powerleft;
    double powerright;

    double launchpower;

    boolean upprevstatelaunchspeed;
    boolean downprevstatelaunchspeed;

    double lastuppresslaunchspeed;
    double lastdownpresslaunchspeed;

    boolean prevstatelaunchtoggle;

    double lastpresslaunchtoggle;

    boolean leftprevstatebeaconhitter;
    boolean rightprevstatebeaconhitter;

    boolean rightout;
    boolean leftout;

    boolean launching;

    double leftMultiplier;
    double rightMultiplier;

    double maxliftpowerup;

    double maxliftpowerdown;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {

        leftMultiplier = 1;
        rightMultiplier = 1;

        telemetry.addData("Status", "Initialized");

        //
        launchLeft = hardwareMap.dcMotor.get("launch_left");
        launchRight = hardwareMap.dcMotor.get("launch_right");

        leftback = hardwareMap.dcMotor.get("drive_left");
        rightback = hardwareMap.dcMotor.get("drive_right");
        nom = hardwareMap.dcMotor.get("nom");
        lift = hardwareMap.dcMotor.get("lift");

        launchpower = .4;

        bpright = hardwareMap.servo.get("leftBeacon");
        bpleft = hardwareMap.servo.get("rightBeacon");

        cbleft = hardwareMap.dcMotor.get("cap_left");
        cbright = hardwareMap.dcMotor.get("cap_right");

        cbstartPosL = cbleft.getCurrentPosition();
        cbstartPosR = cbright.getCurrentPosition();

        upprevstatelaunchspeed = false;
        downprevstatelaunchspeed = false;

        lastuppresslaunchspeed = 0;
        lastdownpresslaunchspeed = 0;

        rightprevstatebeaconhitter = false;
        leftprevstatebeaconhitter = false;
        rightout = false;
        leftout = false;

        //make sure cap ball doesn't go too fast
        maxliftpowerdown = -.3;
        maxliftpowerup = .8;

        //double v = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
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
        double cbEncoderR = cbright.getCurrentPosition() - cbstartPosR;
        double cbEncoderL = cbleft.getCurrentPosition() - cbstartPosL;

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Launch Power:", launchpower);

        telemetry.addData("Right Encoder Value", rightback.getCurrentPosition());
        VoltageSensor v = hardwareMap.voltageSensor.iterator().next();
        telemetry.addData(v.getDeviceName() + " voltage: ", v.getVoltage());
        telemetry.update();

        //Launch
        if (gamepad2.dpad_up == true && upprevstatelaunchspeed == false && runtime.time() - lastuppresslaunchspeed > .5) {
            launchpower += .05;
            upprevstatelaunchspeed = true;
            lastuppresslaunchspeed = runtime.time();
        } else {
            upprevstatelaunchspeed = false;
        }
        if (gamepad2.dpad_down ==  true && downprevstatelaunchspeed == false && runtime.time() - lastdownpresslaunchspeed > .5) {
            launchpower -= .05;
            downprevstatelaunchspeed = true;
            lastdownpresslaunchspeed = runtime.time();
        } else {
            downprevstatelaunchspeed = false;
        }

        if (gamepad1.right_bumper ==  true && prevstatelaunchtoggle == false && runtime.time() - lastpresslaunchtoggle > .5) {
            launching = !launching;
            prevstatelaunchtoggle = true;
            lastpresslaunchtoggle = runtime.time();
        } else {
            prevstatelaunchtoggle = false;
        }


        if (launching == true && gamepad1.left_bumper == false) {
            launchRight.setPower(launchpower);
            launchLeft.setPower(-launchpower);
        } else if (launching == false && gamepad1.left_bumper == true){
            launchRight.setPower(-.2);
            launchLeft.setPower(.2);
        } else {
            launchRight.setPower(0);
            launchLeft.setPower(0);
        }

        launchpower = trim(launchpower);

        //Drive

        powerright = gamepad1.left_stick_y;
        powerleft = gamepad1.left_stick_y;

        powerright += gamepad1.right_stick_x;
        powerleft -= gamepad1.right_stick_x;

        leftback.setPower(-trim(powerleft));
        rightback.setPower(trim(powerright));


        if (gamepad2.a == false && gamepad2.y == true) {
            nom.setPower(1);
        } else if (gamepad2.y == false && gamepad2.a == true) {
            nom.setPower(-1);
        } else {
            nom.setPower(0);
        }

        //Lift
        if (gamepad2.right_bumper == true  && rightprevstatebeaconhitter == false) {
            rightout = !rightout;
            rightprevstatebeaconhitter = true;
        } else {
            rightprevstatebeaconhitter = false;
        }

        if (gamepad2.left_bumper == true && leftprevstatebeaconhitter == false) {
            leftout = !leftout;
            leftprevstatebeaconhitter = true;
        } else {
            leftprevstatebeaconhitter = false;
        }

        if (rightout == true ){
            bpright.setPosition(1.0);
        } else {
            bpright.setPosition(0.0);
        }

        if (leftout == true ) {
            bpleft.setPosition(0.0);
        } else {

            bpleft.setPosition(0.95);
        }

        if (gamepad2.x == true && gamepad2.b == false) {
            lift.setPower(1);
        } else if (gamepad2.x == false && gamepad2.b == true) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        boolean manuallyControlled = false;

//        if (cbleftState == 0 && gamepad2.left_stick_y > 0 && cbleft.getCurrentPosition() > cbstartPosL) {
//            cbleft.setPower(gamepad2.left_stick_y);
//            manuallyControlled = true;
//
//        } else if (cbleftState == 0 && gamepad2.left_stick_y < 0) {
//            cbleft.setPower(gamepad2.left_stick_y);
//            manuallyControlled = true;
//        }
//
//        if (cbrightState == 0 && gamepad2.right_stick_y > 0 && cbright.getCurrentPosition() > cbstartPosR) {
//            cbright.setPower(gamepad2.right_stick_y);
//            manuallyControlled = true;
//        } else if (cbrightState == 0 && gamepad2.right_stick_y < 0) {
//            cbright.setPower(gamepad2.right_stick_y);
//            manuallyControlled = true;
//        }

//        if (!manuallyControlled) {
//            cbleft.setPower(cbleftState * cbpowerMultiplier * leftMultiplier);
//            cbright.setPower(-1 * cbrightState * cbpowerMultiplier * rightMultiplier);
//        }


        if (cbright.getCurrentPosition() > cbstartPosR) {
            if (gamepad2.right_stick_y > maxliftpowerup) {
                cbright.setPower(maxliftpowerup);
            } else if (gamepad2.right_stick_y < maxliftpowerdown) {
                cbright.setPower(maxliftpowerdown);
            } else {
                cbright.setPower(gamepad2.right_stick_y);
            }
        } else {
            cbright.setPower(0);
        }

        if (cbleft.getCurrentPosition() > cbstartPosL) {
            if (gamepad2.left_stick_y > maxliftpowerup) {
                cbleft.setPower(maxliftpowerup);
            } else if (gamepad2.left_stick_y < maxliftpowerdown) {
                cbleft.setPower(maxliftpowerdown);
            } else {
                cbleft.setPower(gamepad2.left_stick_y);
            }
        } else {
            cbleft.setPower(0);
        }
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
