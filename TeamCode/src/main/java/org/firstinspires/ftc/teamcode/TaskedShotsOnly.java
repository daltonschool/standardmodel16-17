package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Tasked Operation - Shots only")
public class TaskedShotsOnly extends TaskedOperation {
    public Alliance getCurrentAlliance() {
        return Alliance.RED;
    }
    public boolean isASpookster() {
        return false;
    }
    public boolean isShotsOnly() {
        return true;
    }
}