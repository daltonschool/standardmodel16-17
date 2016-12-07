package org.firstinspires.ftc.teamcode;

public class PIDController {
    double P; //increase until there is a lot of overshooting
    double D; //increase until the overshooting is reduced to an acceptable level
    double I; //increase until the final error is equal to zero.
    double Kp;
    double Ki;
    double Kd;
    double total_integral;
    double lastErr;
    long previous_time = System.currentTimeMillis();

    public PIDController (double a, double b, double c) {
        P = a;
        I = b;
        D = c;
    }

    public double Step( double cError ) {
        long current_time = System.currentTimeMillis();
        double time_diff = (current_time - previous_time);
        double current_error = cError;
        total_integral += (current_error * time_diff);
        double error_difference = (current_error - lastErr) / time_diff;
        Kp = current_error;
        Ki = total_integral;
        Kd = error_difference;
        double R = P * Kp + I * Ki + D * Kd;
        lastErr = current_error;
        previous_time = current_time;
        if (R > 100) {
            R = 100;
        }
        else if (R < -100) {
            R = -100;
        }

        Robot.telemetry.addData("Kp: ", (Kp));
        Robot.telemetry.addData("Ki: ", (Ki));
        Robot.telemetry.addData("Kd: ", (Kd));
        Robot.telemetry.addData("R: ", R);
        Robot.telemetry.addData("Current Error", current_error);
        return R;
    }
}
