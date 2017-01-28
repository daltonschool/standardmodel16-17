package org.firstinspires.ftc.teamcode.sensors;

public abstract class Sensor {
    public abstract boolean ping();

    public abstract byte firmwareRevision();
    public abstract byte manufacturer();
    public abstract byte sensorIDCode();

    public abstract String name();
    public abstract String uniqueName();

    public abstract void init();
    public abstract void update();
}
