// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorNeo;

public class CommandNeoPosition extends Command {
  private final MotorNeo m_motorNeo;

  /** Creates a new CommandNeoPosition. */
  public CommandNeoPosition(MotorNeo motor_neo) {
    m_motorNeo = motor_neo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motorNeo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_motorNeo.setPositionCtrl(4.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motorNeo.motorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_motorNeo.getPositionError()) < 0.5;
  }
}
