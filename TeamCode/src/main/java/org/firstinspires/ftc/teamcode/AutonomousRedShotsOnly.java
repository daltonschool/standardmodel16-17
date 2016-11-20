package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Operation - RED - SO")
public class AutonomousRedShotsOnly extends AutonomousOperation {
    public Alliance getCurrentAlliance() { return Alliance.RED; }
}
