package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Tasked Operation - RED")
public class TaskedRed extends TaskedOperation {
    public Alliance getCurrentAlliance() {
        return Alliance.RED;
    }
}