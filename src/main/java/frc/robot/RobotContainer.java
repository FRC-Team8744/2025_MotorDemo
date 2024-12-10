// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CommandNeo;
import frc.robot.subsystems.MotorNeo;

import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final MotorNeo m_motorNeo = new MotorNeo();

  // Driver's controller
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  // Objects for Shuffleboard debug display
  // Doc link: https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html#shuffleboard
  private ShuffleboardTab tab = Shuffleboard.getTab("CommandButtons");
  private ShuffleboardContainer shuf_CmdList = tab.getLayout("Commands", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3).withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Add commands to debug display
    shuf_CmdList.add(new CommandNeo(m_motorNeo, Constants.DRIVE_MODE_DUTY_CYCLE));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Schedule `DefaultCommand` when the A button is pressed, canceling on release.
    new JoystickButton(m_driverController, Button.kA.value).whileTrue(new CommandNeo(m_motorNeo, Constants.DRIVE_MODE_DUTY_CYCLE));
    new JoystickButton(m_driverController, Button.kY.value).whileTrue(new CommandNeo(m_motorNeo, Constants.DRIVE_MODE_POSITION));
    new JoystickButton(m_driverController, Button.kX.value).whileTrue(new CommandNeo(m_motorNeo, Constants.DRIVE_MODE_LIMITED_POSITION));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
