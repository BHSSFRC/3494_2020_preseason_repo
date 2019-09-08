package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavX {

    private static NavX INSTANCE = new NavX();
    private AHRS ahrs;
    private double resetValue;
    private double resetPitchValue;
    private double resetYawValue;

    private NavX() {
        this.ahrs = new AHRS(SPI.Port.kMXP);
        this.resetFusedHeading();
        this.resetPitch();
        this.resetYaw();
    }

    public static NavX getInstance() {
        return INSTANCE;
    }

    public void resetPitch() {
        this.resetPitchValue = -this.ahrs.getPitch();
    }
    public void resetYaw() {
        this.resetYawValue = this.ahrs.getYaw();
    }

    public double getFusedHeading() {
        double fusedHeading = (ahrs.getFusedHeading() * (Math.PI / 180) - resetValue);
        if (fusedHeading < 0) {
            return 2 * Math.PI + fusedHeading;
        }
        return fusedHeading;
    }

    public void resetFusedHeading() {
        resetValue = ahrs.getFusedHeading();
    }

    public double getPitchDegrees() {
        double pitchDegrees = -this.ahrs.getPitch() - resetPitchValue;
        return pitchDegrees;
    }

    public double getYawDegrees() {
        double yawDegrees = this.ahrs.getYaw() - resetYawValue;
        return yawDegrees;
    }

    public double getRollDegrees() {
        return this.ahrs.getRoll();
    }

    /*
     * periodic had to be commented out because NavX was moved to sensors.
     * This code is saved so it can be reused.
     *
     * public void periodic() {
     * if (SmartDashboard.getBoolean("Display navX data?", false)) {
     * System.out.println("The robot angle is " + this.getFusedHeading());
     * }
     * }
     */
}

