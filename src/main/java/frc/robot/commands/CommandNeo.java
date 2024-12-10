// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.MotorNeo;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CommandNeo extends Command {
  private MotorNeo m_motorNeo;
  private int m_mode;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CommandNeo(MotorNeo motor_neo, int mode) {
    m_motorNeo = motor_neo;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motorNeo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_mode == Constants.DRIVE_MODE_DUTY_CYCLE) {
      m_motorNeo.spinMotor(0.1);
    } else if (m_mode == Constants.DRIVE_MODE_POSITION) {
      m_motorNeo.setPositionCtrl(m_motorNeo.getPositionGoal());
    } else if (m_mode == Constants.DRIVE_MODE_LIMITED_POSITION) {
      m_motorNeo.setPositionCtrlLimited(m_motorNeo.getPositionGoal());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_motorNeo.motorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDone = false;

    if (m_mode != Constants.DRIVE_MODE_DUTY_CYCLE) {
      isDone = Math.abs(m_motorNeo.getPositionError()) < 0.5;
    }

    return isDone;
  }
}
