package org.firstinspires.ftc.teamcode;

public class PIDController {
    double P = .15; //increase until there is a lot of overshooting
    double D = 0; //increase until the overshooting is reduced to an acceptable level
    double I = 0; //increase until the final error is equal to zero.
    double Kp;
    double Ki;
    double Kd;
    double total_integral;
    double lastErr;
    long previous_time = System.currentTimeMillis();

    public double Step( double cError ) {
        long current_time = System.currentTimeMillis();
        double time_diff = (current_time - previous_time);
        double current_error = cError;
        total_integral += (current_error * time_diff);
        double error_difference = (current_error - lastErr) / time_diff;
        double R = P * current_error + I * total_integral + D * error_difference;
        lastErr = current_error;
        previous_time = current_time;
        if (R > 100) {
            R = 100;
        }
        else if (R < -100) {
            R = -100;
        }
        Robot.telemetry.addData("Kp: ", (Kp * P));
        Robot.telemetry.addData("Ki: ", (Ki * I));
        Robot.telemetry.addData("Kd: ", (Kd * D));
        Robot.telemetry.addData("R: ", R);
        Robot.telemetry.addData("Current Error", current_error);
        return R;
    }
}
