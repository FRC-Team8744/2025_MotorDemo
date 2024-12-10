// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int kDebugLevel = 3; // 0 = None, 1 = Errors, 2 = Info, 3 = Debug

  public static final int NO_DEBUG_INFO = 0;
  public static final int DEBUG_LEVEL_ERRORS = 1;
  public static final int DEBUG_LEVEL_INFO = 2;
  public static final int DEBUG_LEVEL_ALL = 3;
  
  public static final int kMaxSpeedPercentAuto = 100;
  public static final int kMaxSpeedPercentTeleop = 100;

  public static final int DRIVE_MODE_DUTY_CYCLE = 0;
  public static final int DRIVE_MODE_POSITION = 1;
  public static final int DRIVE_MODE_VELOCITY = 2;
  public static final int DRIVE_MODE_LIMITED_POSITION = 3;

  public static final class MechanismConstants {
    public static final int kMotorNeoAddress = 8;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
