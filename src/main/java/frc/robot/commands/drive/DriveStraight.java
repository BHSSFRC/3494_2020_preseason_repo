package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.sensors.NavX;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.QuadTimer;
import frc.robot.util.SynchronousPIDF;

import static frc.robot.commands.drive.Drive.setCamera;

public class DriveStraight extends Command {

    /**
     * Indicator of which side of the robot is the "front" for operator driving. Set to true iff the driver has
     * swapped fronts. Use for applications beyond operator input is strictly forbidden for sanity.
     */
    private boolean sideFlipped = false;

    private SynchronousPIDF pidController;

    private QuadTimer m_timer;

    public DriveStraight() {
        requires(Drivetrain.getInstance());

        //takes same PID constants as DriveAntitipPD does. if necessary will add new set of constants
        pidController = new SynchronousPIDF(RobotMap.DRIVE.KP, RobotMap.DRIVE.KI, RobotMap.DRIVE.KD);

        pidController.setInputRange(0, 2 * Math.PI);
        pidController.setContinuous(true);
        pidController.setOutputRange(-1, 1);

        this.m_timer = new QuadTimer();
    }

    @Override
    protected void initialize(){
        pidController.setSetpoint(NavX.getInstance().getFusedHeading() * Math.PI / 180);
        this.m_timer.start();
    }

    @Override
    protected void execute() {
        double[] stickSpeeds = {RobotMap.DRIVE.DRIVE_STRAIGHT_POWER, RobotMap.DRIVE.DRIVE_STRAIGHT_POWER};
        double correctionAmount = 0;

        double pidOutput = this.pidController.calculate(NavX.getInstance().getFusedHeading() * Math.PI / 180, this.m_timer.delta());

        stickSpeeds[0] += pidOutput;
        stickSpeeds[1] -= pidOutput;
        
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
    }
}
