package frc.robot.commands.drive;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.sensors.NavX;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.QuadTimer;
import frc.robot.util.SynchronousPIDF;

public class DriveStraight extends Drive {

    /**
     * Indicator of which side of the robot is the "front" for operator driving. Set to true iff the driver has
     * swapped fronts. Use for applications beyond operator input is strictly forbidden for sanity.
     */
    private boolean sideFlipped = false;

    private double yawDegrees;

    private SynchronousPIDF pidController;

    private QuadTimer m_timer;
    private double lastTime;

    public DriveStraight() {
        requires(Drivetrain.getInstance());

        //takes same PID constants as DriveAntitipPD does. if necessary will add new set of constants
        pidController = new SynchronousPIDF(RobotMap.DRIVE.KP, RobotMap.DRIVE.KI, RobotMap.DRIVE.KD);
        pidController.setSetpoint(0);

        pidController.setInputRange(-180, 180);
        pidController.setContinuous(true);
        pidController.setOutputRange(-1, 1);

        this.m_timer = new QuadTimer();
    }


    private void updateFusedHeading() {
        this.yawDegrees = NavX.getInstance().getFusedHeading();
    }


    @Override
    protected void execute() {
        double[] stickSpeeds = {RobotMap.DRIVE.DRIVE_STRAIGHT_POWER, RobotMap.DRIVE.DRIVE_STRAIGHT_POWER};
        this.updateFusedHeading();
        double correctionAmount = 0;

        double pidOutput = this.pidController.calculate(yawDegrees, this.m_timer.get() - this.lastTime);
        correctionAmount = pidOutput * RobotMap.DRIVE.PID_YAW_CORRECTION_FACTOR;

        stickSpeeds[0] += correctionAmount;
        stickSpeeds[1] -= correctionAmount;
        stickSpeeds = this.normalize(stickSpeeds);


        if (!sideFlipped) {
            Drivetrain.getInstance().tankDrive(stickSpeeds[0], stickSpeeds[1]);
        } else {
            Drivetrain.getInstance().tankDrive(-stickSpeeds[1], -stickSpeeds[0]);
        }

        int pov = OI.getInstance().getLeftPOV();
        if (pov != -1) {
            if (pov == 0) {
                this.sideFlipped = false;
                setCamera("RPI");
            } else if (pov == 180) {
                this.sideFlipped = true;
                setCamera("USB");
            }
        }
        this.lastTime = this.m_timer.get();
    }
}
