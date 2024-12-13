// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.MotorNeo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class bobik extends SequentialCommandGroup {
  /** Creates a new bobik. */
  public bobik(MotorNeo Bobikko) {
    addCommands(new CommandNeo(Bobikko, Constants.DRIVE_MODE_LIMITED_POSITION));
  }
}