package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class QuadTimer extends Timer {
    private double lastTime;

    public double delta(){
        double delta = this.get() - this.lastTime;
        this.lastTime = this.get();
        return delta;
    }

}
