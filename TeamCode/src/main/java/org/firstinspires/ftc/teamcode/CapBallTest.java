package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CapBallTest", group = "TeleOp")
public class CapBallTest extends OpMode {

    DcMotor cbleft;
    DcMotor cbright;

    double maxliftpowerup = 0.8;
    double maxliftpowerdown = -0.3;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        cbleft = hardwareMap.dcMotor.get("cap_left");
        cbright = hardwareMap.dcMotor.get("cap_right");

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
        cbright.setPower(scaletobounds(-gamepad2.right_stick_y, -maxliftpowerdown, -maxliftpowerup));
        cbleft.setPower(scaletobounds(gamepad2.left_stick_y, maxliftpowerup, maxliftpowerdown));
    }


    public double trim (double number) {
        if (number > 1) {
            number = 1;
        } else if (number < -1) {
            number = -1;
        }
        return number;
    }

    public double scaletobounds(double input, double max, double min) {
        double output = input;
        if (input > 0) {
            output =+ input*max;
        } else if (input < 0) {
            output =- input*min;
        }
        return output;
    }
}
