package org.firstinspires.ftc.teamcode.taskutil;

public abstract class Task {
    public Object extra;

    public Task(Object e) {
        extra = e;
    }

    public abstract void init();
    public abstract void run() throws InterruptedException;
}
