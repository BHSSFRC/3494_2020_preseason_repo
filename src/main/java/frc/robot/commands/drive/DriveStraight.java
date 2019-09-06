package frc.robot.commands.drive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.sensors.NavX;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SynchronousPIDF;

public class DriveStraight extends Command {

    /**
     * Indicator of which side of the robot is the "front" for operator driving. Set to true iff the driver has
     * swapped fronts. Use for applications beyond operator input is strictly forbidden for sanity.
     */
    private boolean sideFlipped = false;

    private double yawDegrees;

    private SynchronousPIDF pidController;

    private Timer m_timer;
    private double lastTime;

    public DriveStraight() {
        requires(Drivetrain.getInstance());

        //takes same PID constants as DriveAntitipPD does. if necessary will add new set of constants
        pidController = new SynchronousPIDF(RobotMap.DRIVE.KP, RobotMap.DRIVE.KI, RobotMap.DRIVE.KD);
        pidController.setSetpoint(0);

        //input range isn't 0 to 360 because negative yaw angles from the gyro exist?
        pidController.setInputRange(-180, 180);
        pidController.setContinuous(true);
        pidController.setOutputRange(-1, 1);

        this.m_timer = new Timer();
    }

    /**
     * @param motorSpeeds array of motor power values
     *                    If any of the values are more than 1, they aren't valid values for motor power.
     *                    If so, it divides all array values by the largest value to preserve the value ratios while making them valid motor power values.
     */
    private double[] normalize(double[] motorSpeeds) {
        double max = Math.abs(motorSpeeds[0]);
        boolean normFlag = max > 1;

        for (int i = 1; i < motorSpeeds.length; i++) {
            if (Math.abs(motorSpeeds[i]) > max) {
                max = Math.abs(motorSpeeds[i]);
                normFlag = max > 1;
            }
        }

        if (normFlag) {
            for (int i = 0; i < motorSpeeds.length; i++) {
                motorSpeeds[i] /= max;
            }
        }
        return motorSpeeds;
    }



    private void updateYawStatus() {
        this.yawDegrees = NavX.getInstance().getYawDegrees();
    }

    @Override
    protected void initialize() {
        setCamera("RPI");
    }

    @Override
    protected void execute() {
        //TODO: speeds should just be some constant
        double[] stickSpeeds = {RobotMap.DRIVE.DRIVE_STRAIGHT_POWER, RobotMap.DRIVE.DRIVE_STRAIGHT_POWER};
        this.updateYawStatus();
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

    @Override
    protected boolean isFinished() {
        return false;
    }

    private static boolean setCamera(String camera) {
        NetworkTable engineering = NetworkTableInstance.getDefault().getTable("engineering");
        return engineering.getEntry("camera").setString(camera);
    }
}
