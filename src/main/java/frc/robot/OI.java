/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.climb.Shift;
import frc.robot.commands.hatch.ExtendHatchManipulator;
import frc.robot.commands.hatch.RetractHatchManipulator;
import frc.robot.commands.hatch.SetHatchExtender;

public class OI {

    private static OI INSTANCE = new OI();
    private Joystick leftFlight;
    private Joystick rightFlight;
    private XboxController xbox;

    private JoystickButton extendHatchManipulatorButton;
    private JoystickButton retractHatchManipulatorButton;
    private JoystickButton extendHatcher;
    private JoystickButton retractHatcher;

    private JoystickButton engageButton;
    private JoystickButton disengageButton;

    private OI() {
        leftFlight = new Joystick(RobotMap.OI.LEFT_JOY);
        rightFlight = new Joystick(RobotMap.OI.RIGHT_JOY);
        xbox = new XboxController(RobotMap.OI.XBOX);

        // Xbox binds
        extendHatchManipulatorButton = new JoystickButton(xbox, RobotMap.OI.EJECT_HATCH);
        retractHatchManipulatorButton = new JoystickButton(xbox, RobotMap.OI.RESET_EJECTOR);
        extendHatcher = new JoystickButton(xbox, RobotMap.OI.EXTEND_HATCHER);
        retractHatcher = new JoystickButton(xbox, RobotMap.OI.RETRACT_HATCHER);
        extendHatchManipulatorButton.whenPressed(new ExtendHatchManipulator());
        retractHatchManipulatorButton.whenPressed(new RetractHatchManipulator());
        extendHatcher.whenPressed(new SetHatchExtender(true));
        retractHatcher.whenPressed(new SetHatchExtender(false));
        // Driver joystick binds
        disengageButton = new JoystickButton(leftFlight, RobotMap.OI.SHIFT_DISENGAGE_BUTTON);
        engageButton = new JoystickButton(leftFlight, RobotMap.OI.SHIFT_ENGAGE_BUTTON);
        disengageButton.whenPressed(new Shift(DoubleSolenoid.Value.kForward));
        engageButton.whenPressed(new Shift(DoubleSolenoid.Value.kReverse));
    }

    public static double removeDeadband(double y) {
        if (Math.abs(y) <= .05) {
            return 0;
        } else {
            return y;
        }
    }

    public double getLeftY() {
        return -removeDeadband(leftFlight.getY());
    }

    public double getRightY() {
        return -removeDeadband(rightFlight.getY());
    }

    public boolean getXboxLeftBumper() {
        return this.xbox.getBumper(GenericHID.Hand.kLeft);
    }

    public boolean getXboxRightBumper() {
        return this.xbox.getBumper(GenericHID.Hand.kRight);
    }

    public boolean getXboxA() {
        return this.xbox.getAButton();
    }

    public boolean getXboxB() {
        return this.xbox.getBButton();
    }

    public static OI getInstance() {
        return INSTANCE;
    }

    @Override
    protected Object clone() throws CloneNotSupportedException {
        throw new CloneNotSupportedException();
    }
}
