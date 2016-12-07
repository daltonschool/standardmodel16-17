package org.firstinspires.ftc.teamcode;

import com.google.gson.Gson;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;

public class Utils {

    /**
     * Shows a toast message on the Driver Station.
     * @param message The message to show.
     * @param duration Probably the duration of the message. Haven't tried with non-zero values though.
     */
    public static void showToast(String message, int duration) {
        ShowToastExtra showToastExtra = new ShowToastExtra(message, duration);
        Gson g = new Gson();
        Command toastCmd = new Command("CMD_SHOW_TOAST", g.toJson(showToastExtra));
        NetworkConnectionHandler.getInstance().sendCommand(toastCmd);
    }

    public static Orientation matrixToOrientation(OpenGLMatrix location) {
        return Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }
}

class ShowToastExtra {
    public String message;
    public int duration;

    public ShowToastExtra(String m, int d) {
        this.message = m;
        this.duration = d;
    }
}