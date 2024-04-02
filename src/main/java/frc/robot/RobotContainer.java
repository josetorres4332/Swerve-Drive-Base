// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController m_controller;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final HashMap<String, Command> eventMap = new HashMap<>();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {

         /************************Joystick Axis and Buttons ************************
         *                                  AXISES
         * Left X Stick LEFT/RIGHT = 0                     Left Y Stick UP/DOWN = 1
         * Left Trigger = 2                                Right Trigger = 3
         * Right X Stick LEFT/RIGHT = 4                    Right Y Stick LEFT/RIGHT = 5 
         */

    m_controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> driverJoystick.getRawAxis(OIConstants.kDriverLeftRotAxis),
                                () -> driverJoystick.getRawAxis(OIConstants.kDriverRightRotAxis),
                                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    configureBindings();

  }

  private void configureBindings() {

        /*                              BUTTONS
         * Button A = 0                                    Button B = 1
         * Button X = 2                                    Button Y = 3
         * Left Bumper = 4                                 Right Bumper = 5
         * Pause Button = 6                                Menu Button = 7  
         */

    new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonIdx).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new JoystickButton(operatorJoystick, OperatorConstants.kOperatorResetGyroButtonIdx).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

  }

  public Command getAutonomousCommand() {

    return m_chooser.getSelected();

  }
}
