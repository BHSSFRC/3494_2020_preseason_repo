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

public class DriveAntitipPD extends Command {

    /**
     * Indicator of which side of the robot is the "front" for operator driving. Set to true iff the driver has
     * swapped fronts. Use for applications beyond operator input is strictly forbidden for sanity.
     */
    private boolean sideFlipped = false;

    private double pitchDegrees;

    private SynchronousPIDF m_pidController;

    private Timer m_timer;
    private double m_lastTime = 0;

    public DriveAntitipPD() {
        requires(Drivetrain.getInstance());

        this.m_timer = new Timer();

        m_pidController = new SynchronousPIDF(RobotMap.DRIVE.KP, RobotMap.DRIVE.KI, RobotMap.DRIVE.KD);
        m_pidController.setSetpoint(0);
        m_pidController.setInputRange(0, 360);
        m_pidController.setContinuous(true);
        m_pidController.setOutputRange(-1, 1);
    }

    private static double powerCurve(double x) {
        // https://www.desmos.com/calculator/g07ukjj7bl
        double curve = (0.5D * (Math.atan(Math.PI * (Math.abs(x) - 0.5D)))) + 0.5D;
        return Math.copySign(curve, x);
    }

    /**
     * @param motorSpeeds array of motor power values
     *                    If any of the values are more than 1, they aren't valid values for motor power.
     *                    If so, it divides all array values by the largest value to preserve the value ratios while making them valid motor power values.
     */
    private void normalize(double[] motorSpeeds) {
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
    }

    private void correctForPitch(double[] stickSpeeds) {//x-tip
        //if its over 45 there's no point in correcting it
        if (Math.abs(pitchDegrees) < 45) {
            //correctionFactor keeps the tilt correction within a certain threshold so it doesn't correct too much

            double correctionOffset = RobotMap.DRIVE.CORRECTION_FACTOR * (pitchDegrees - RobotMap.DRIVE.PITCH_THRESHOLD_DEGREES);
            //double correctionOffset = this.pitchDegrees / 10;
            if (Math.abs(pitchDegrees) < RobotMap.DRIVE.PITCH_ALARM_THRESHOLD) {
                stickSpeeds[0] += correctionOffset;
                stickSpeeds[1] += correctionOffset;
            } else {
                stickSpeeds[0] = correctionOffset;
                stickSpeeds[1] = correctionOffset;
            }
            normalize(stickSpeeds);
        }
    }

    private void updatePitchStatus() {
        this.pitchDegrees = NavX.getInstance().getPitchDegrees();
    }

    @Override
    protected void initialize() {
        setCamera("RPI");
        this.m_timer.start();
    }

    @Override
    protected void execute() {
        double leftRaw = OI.getInstance().lowPower() ? OI.getInstance().getLeftY() / 2.0 : OI.getInstance().getLeftY();
        double rightRaw = OI.getInstance().lowPower() ? OI.getInstance().getRightY() / 2.0 : OI.getInstance().getRightY();
        double[] stickSpeeds = {powerCurve(leftRaw), powerCurve(rightRaw)};
        double correction = 0;


        if (!Drivetrain.getInstance().getIsAntiTipDisabled()) {
            updatePitchStatus();
            if (Math.abs(pitchDegrees) > RobotMap.DRIVE.PITCH_THRESHOLD_DEGREES) {
                //this.correctForPitch(stickSpeeds);
                correction = this.m_pidController.calculate(pitchDegrees, this.m_timer.get() - this.m_lastTime);
            }
        }


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

        this.m_lastTime = this.m_timer.get();
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
