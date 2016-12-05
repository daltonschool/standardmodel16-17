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
    int SampleTime = 1000; //1 sec

    public void Compute(double cError) {
        long current_time = System.currentTimeMillis();

        int timeChange = safeLongToInt(current_time - previous_time);
        if (timeChange >= SampleTime) {
            /*Compute all the working error variables*/
            double time_diff = (current_time - previous_time);
            double current_error = cError;
            total_integral += cError;
            double error_difference = (current_error - lastErr);
            Kp = current_error;
            Ki = total_integral;
            Kd = error_difference;

            /*Compute PID Output*/
            double R = P * Kp + I * Ki + D * Kd;

            /*Remember some variables for next time*/
            lastErr = cError;
            previous_time = current_time;
        }
    }

    public void SetTunings(double a, double b, double c) {
        P = a;
        I = b;
        D = c;
    }

    public void SetSampleTime(int NewSampleTime) {
        if (NewSampleTime > 0) {
            double ratio = (double) NewSampleTime / (double) SampleTime;
            I *= ratio;
            D /= ratio;
            SampleTime = NewSampleTime;
        }
    }

    public static int safeLongToInt(long l) {
        if (l < Integer.MIN_VALUE || l > Integer.MAX_VALUE) {
            throw new IllegalArgumentException
                    (l + " cannot be cast to int without changing its value.");
        }
        return (int) l;
    }
}
//
//        double error_difference = (current_error - lastErr) / time_diff;
//        Kp = current_error;
//        Ki = total_integral;
//        Kd = error_difference;
//        double R = P * Kp + I * Ki + D * Kd;
//        lastErr = current_error;
//        previous_time = current_time;
//        if (R > 1) {
//            R = 1;
//        }
//        else if (R < -1) {
//            R = -1;
//        }
//
//        Robot.telemetry.addData("Kp: ", (Kp));
//        Robot.telemetry.addData("Ki: ", (Ki));
//        Robot.telemetry.addData("Kd: ", (Kd));
//        Robot.telemetry.addData("R: ", R);
//        Robot.telemetry.addData("Current Error", current_error);
//        return R;
//    }
//}
