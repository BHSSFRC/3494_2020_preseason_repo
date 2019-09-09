package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drive.Drive;
import frc.robot.sensors.HRLVMaxSonar;
import frc.robot.sensors.NavX;
import frc.robot.sensors.PDP;

public class Drivetrain extends PIDSubsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /**
     * Master CANSparkMax, left side.
     */
    private CANSparkMax driveLeftMaster;
    /**
     * Follower CANSparkMax, left side.
     */
    private CANSparkMax driveLeftFollowOne;
    /**
     * Additional follower CANSparkMax, left side.
     */
    private CANSparkMax driveLeftFollowTwo;

    /**
     * Master CANSparkMax, right side.
     */
    private CANSparkMax driveRightMaster;
    /**
     * Follower CANSparkMax, right side.
     */
    private CANSparkMax driveRightFollowOne;
    /**
     * Additional follower CANSparkMax, right side.
     */
    private CANSparkMax driveRightFollowTwo;

    private CANEncoder encLeftMaster;
    private CANEncoder encLeftFollowOne;
    private CANEncoder encLeftFollowTwo;

    private CANEncoder encRightMaster;
    private CANEncoder encRightFollowOne;
    private CANEncoder encRightFollowTwo;

    private double pidOutput = 0;

    private HRLVMaxSonar ultrasonic;

    private static Drivetrain INSTANCE = new Drivetrain();

    private boolean isAntiTipDisabled = true;

    private Drivetrain() {
        super("Drivetrain", 1.0, 0, 0);

        CANSparkMax[] motors = new CANSparkMax[6];
        int i = 0;

        this.driveLeftMaster = new CANSparkMax(RobotMap.DRIVETRAIN.LEFT_MASTER_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.driveLeftMaster.setInverted(true);
        motors[i] = this.driveLeftMaster;
        i += 1;
        this.encLeftMaster = this.driveLeftMaster.getEncoder();

        this.driveLeftFollowOne = new CANSparkMax(RobotMap.DRIVETRAIN.LEFT_FOLLOWER_ONE_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.driveLeftFollowOne.follow(driveLeftMaster);
        this.driveLeftFollowOne.setInverted(true);
        motors[i] = this.driveLeftFollowOne;
        i += 1;
        this.encLeftFollowOne = this.driveLeftFollowOne.getEncoder();

        this.driveLeftFollowTwo = new CANSparkMax(RobotMap.DRIVETRAIN.LEFT_FOLLOWER_TWO_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.driveLeftFollowTwo.follow(driveLeftMaster);
        this.driveLeftFollowTwo.setInverted(true);
        motors[i] = this.driveLeftFollowTwo;
        i += 1;
        this.encLeftFollowTwo = this.driveLeftFollowTwo.getEncoder();

        this.driveRightMaster = new CANSparkMax(RobotMap.DRIVETRAIN.RIGHT_MASTER_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
        motors[i] = this.driveRightMaster;
        i += 1;
        this.encRightMaster = this.driveRightMaster.getEncoder();

        this.driveRightFollowOne = new CANSparkMax(RobotMap.DRIVETRAIN.RIGHT_FOLLOWER_ONE_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.driveRightFollowOne.follow(driveRightMaster);
        motors[i] = this.driveRightFollowOne;
        i += 1;
        this.encRightFollowOne = this.driveRightFollowOne.getEncoder();

        this.driveRightFollowTwo = new CANSparkMax(RobotMap.DRIVETRAIN.RIGHT_FOLLOWER_TWO_CHANNEL, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.driveRightFollowTwo.follow(driveRightMaster);
        motors[i] = this.driveRightFollowTwo;
        i += 1;
        this.encRightFollowTwo = this.driveRightFollowTwo.getEncoder();

        for (CANSparkMax m : motors) {
            m.setSmartCurrentLimit(RobotMap.DRIVETRAIN.CURRENT_LIMIT);
        }

        this.ultrasonic = new HRLVMaxSonar(1);

        SmartDashboard.putBoolean("Anti-Tip Enabled", !this.isAntiTipDisabled);
    }

    /**
     * Tank drive.
     *
     * @param leftSpeed  Speed of left side.
     * @param rightSpeed Speed of right side.
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        double[] motorSpeeds = this.normalize(new double[]{leftSpeed, rightSpeed});
        this.driveLeftMaster.set(motorSpeeds[0]);
        this.driveRightMaster.set(motorSpeeds[1]);
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

    //returns feet per sec velocity
    public double getLeftAverageVelocity() {
        double sum = this.encLeftMaster.getVelocity() + this.encLeftFollowOne.getVelocity() + this.encLeftFollowTwo.getVelocity();
        return sum / 3 * RobotMap.DRIVETRAIN.GEAR_RATIO * RobotMap.DRIVETRAIN.WHEEL_CIRCUMFERENCE / 60;
    }

    //returns feet per sec velocity
    public double getRightAverageVelocity() {
        double sum = this.encRightMaster.getVelocity() + this.encRightFollowOne.getVelocity() + this.encRightFollowTwo.getVelocity();
        return sum / 3 * RobotMap.DRIVETRAIN.GEAR_RATIO * RobotMap.DRIVETRAIN.WHEEL_CIRCUMFERENCE / 60;
    }

    //returns in feet
    public double getLeftAveragePosition() {
        double sum = this.encLeftMaster.getPosition() + this.encLeftFollowOne.getPosition() + this.encLeftFollowTwo.getPosition();
        return sum / 3 * RobotMap.DRIVETRAIN.GEAR_RATIO * RobotMap.DRIVETRAIN.WHEEL_CIRCUMFERENCE;
    }

    //returns in feet
    public double getRightAveragePosition() {
        double sum = this.encRightMaster.getPosition() + this.encRightFollowOne.getPosition() + this.encRightFollowTwo.getPosition();
        return sum / 3 * RobotMap.DRIVETRAIN.GEAR_RATIO * RobotMap.DRIVETRAIN.WHEEL_CIRCUMFERENCE;
    }

    public double getLeftMasterCurrent() {
        return this.driveLeftMaster.getOutputCurrent();
    }

    public double getLeftFollowOneCurrent() {
        return this.driveLeftFollowOne.getOutputCurrent();
    }

    public double getLeftFollowTwoCurrent() {
        return this.driveLeftFollowTwo.getOutputCurrent();
    }

    public double getRightMasterCurrent() {
        return this.driveRightMaster.getOutputCurrent();
    }

    public double getRightFollowOneCurrent() {
        return this.driveRightFollowOne.getOutputCurrent();
    }

    public double getRightFollowTwoCurrent() {
        return this.driveRightFollowTwo.getOutputCurrent();
    }

    public CANEncoder getLeftEncoder() {
        return this.driveLeftMaster.getEncoder();
    }

    public CANEncoder getRightEncoder() {
        return this.driveLeftMaster.getEncoder();
    }

    public double getUltrasonicDistance() {
        return this.ultrasonic.getDistance();
    }

    public boolean getIsAntiTipDisabled() {
        return this.isAntiTipDisabled;
    }

    public void toggleAntiTip() {
        this.isAntiTipDisabled = !this.isAntiTipDisabled;
        SmartDashboard.putBoolean("Anti-Tip Enabled", !this.isAntiTipDisabled);
    }

    public void stop() {
        this.tankDrive(0, 0);
    }

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean("Display Drivetrain data?", false)) {
            System.out.println("The left side: " + PDP.getInstance().getCurrent(RobotMap.DRIVETRAIN.LEFT_MASTER_CHANNEL) + ", "
                    + PDP.getInstance().getCurrent(RobotMap.DRIVETRAIN.LEFT_FOLLOWER_ONE_CHANNEL) + ", " +
                    PDP.getInstance().getCurrent(RobotMap.DRIVETRAIN.LEFT_FOLLOWER_TWO_CHANNEL));
            System.out.println("The left side: " + PDP.getInstance().getCurrent(RobotMap.DRIVETRAIN.LEFT_MASTER_CHANNEL) + ", "
                    + PDP.getInstance().getCurrent(RobotMap.DRIVETRAIN.LEFT_FOLLOWER_ONE_CHANNEL) + ", " +
                    PDP.getInstance().getCurrent(RobotMap.DRIVETRAIN.LEFT_FOLLOWER_TWO_CHANNEL));
        }
    }

    public static Drivetrain getInstance() {
        return INSTANCE;
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new Drive());
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        throw new CloneNotSupportedException();
    }

    @Override
    protected double returnPIDInput() {
        return NavX.getInstance().getFusedHeading();
    }

    @Override
    protected void usePIDOutput(double output) {
        this.pidOutput = output;
    }

    public double getPidOutput() {
        return this.pidOutput;
    }
}
