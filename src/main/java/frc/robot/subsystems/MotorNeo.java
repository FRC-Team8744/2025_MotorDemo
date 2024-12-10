// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;

public class MotorNeo extends SubsystemBase {
  // Define variables that exist everywhere in the class
  private CANSparkMax MotorDriver;  // The main motor object
  private RelativeEncoder Encoder;  // A separate object for measuring the motor position
  private SparkPIDController Ctrl;  // Object containing the PID controller and settings

  // Objects for Shuffleboard debug display
  // Doc link: https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html#shuffleboard
  private ShuffleboardTab tab = Shuffleboard.getTab("MotorNeo");
  private GenericEntry shuf_posGoal = tab.add("Position Goal", 0).withPosition(0, 0).withSize(1, 1).getEntry();
  private GenericEntry shuf_velGoal = tab.add("Velocity Goal", 0).withPosition(0, 1).withSize(1, 1).getEntry();
  // PID subsection
  private ShuffleboardContainer shuf_pidValues = tab.getLayout("PID Values", BuiltInLayouts.kList).withPosition(1, 0).withSize(2, 3);
  private GenericEntry shuf_pidEnable = shuf_pidValues.add("Enable Update", 0).withSize(2, 1).getEntry();
  private GenericEntry shuf_pidP = shuf_pidValues.add("Proportional", 0).withSize(2, 1).getEntry();
  private GenericEntry shuf_pidI = shuf_pidValues.add("Integral", 0).withSize(2, 1).getEntry();
  private GenericEntry shuf_pidD = shuf_pidValues.add("Derivative", 0).withSize(2, 1).getEntry();
  private GenericEntry shuf_pidIz = shuf_pidValues.add("Int Zone", 0).withSize(2, 1).getEntry();
  private GenericEntry shuf_pidFF = shuf_pidValues.add("Feed Forward", 0).withSize(2, 1).getEntry();
  private GenericEntry shuf_pidMinOut = shuf_pidValues.add("Minimum Output", -1).withSize(2, 1).getEntry();
  private GenericEntry shuf_pidMaxOut = shuf_pidValues.add("Maximum Output", 1).withSize(2, 1).getEntry();
  // Motor subsection
  private ShuffleboardContainer shuf_motorValues = tab.getLayout("Motor Values", BuiltInLayouts.kList).withPosition(3, 0).withSize(2, 3);
  private GenericEntry shuf_position = shuf_motorValues.add("Position", 0).withSize(2, 1).getEntry();
  private GenericEntry shuf_velocity = shuf_motorValues.add("Velocity", 0).withSize(2, 1).getEntry();
  // Limit subsection
  private ShuffleboardContainer shuf_limitValues = tab.getLayout("Limit Values", BuiltInLayouts.kList).withPosition(5, 0).withSize(2, 3);
  private GenericEntry shuf_maxVel = shuf_limitValues.add("Max Velocity", 0).withSize(2, 1).getEntry();
  private GenericEntry shuf_maxAcc = shuf_limitValues.add("Max Acceleration", 0).withSize(2, 1).getEntry();

  // Variables to track changes to shuffleboard
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
  private double DebugP, DebugI, DebugD, DebugIz, DebugFF, DebugMaxOut, DebugMinOut, DebugMaxVel, DebugMaxAcc;
  private double setpoint;
  private double goal_pos;

  // Creates a new motor driver object
  public MotorNeo() {
    // Creating the motor objects here, instead of in the class definition, will allow re-use of the object for multiple motors
    MotorDriver = new CANSparkMax(MechanismConstants.kMotorNeoAddress, MotorType.kBrushless);
    MotorDriver.restoreFactoryDefaults();  // Clear any old programs
    MotorDriver.setSmartCurrentLimit(10);

    Encoder = MotorDriver.getEncoder();  // Assign location of the encoder inside the motor driver object
    Ctrl = MotorDriver.getPIDController();  // Assign location of the PID controller inside the motor driver object

    Encoder.setPosition(0.0);  // Initialize the encoder position to zero on every program load

    // Set PID defaults
    // TODO: Move to constants
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    Ctrl.setP(kP);
    Ctrl.setI(kI);
    Ctrl.setD(kD);
    Ctrl.setFF(kIz);
    Ctrl.setIZone(kFF);  // Controller error must be in this range before integrator is used
    Ctrl.setOutputRange(kMinOutput, kMaxOutput);
    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;
    Ctrl.setSmartMotionMaxVelocity(maxVel, 0);
    Ctrl.setSmartMotionMaxAccel(maxAcc, 0);
    Ctrl.setSmartMotionAllowedClosedLoopError(1.0/40, 0);

    // Set default values in Shuffleboard
    updateShuffleBoardValues(true);
  }

  /**
   * Request the encoder position value for this motor.
   *
   * @return motor position value in rotations
   */
  public double getMotorPosition() {
    // Read encoder value and return it to the caller.
    return Encoder.getPosition();
  }

  public double getPositionGoal() {
    return goal_pos;
  }

  // Add code to request velocity here

  /**
   * Set the reference for position control.
   */
  public void setPositionCtrl(double position) {
    setpoint = position;
    Ctrl.setReference(position, CANSparkMax.ControlType.kPosition);

    /* NOTE:
     * This is equivalent to:
     * MotorDriver.set(speed);
     * 
     * But it has the advantage of canceling any PID control
     */
  }

  // Add code to request limited velocity here

  /**
   * Set the reference for position control.
   */
  public void setPositionCtrlLimited(double position) {
    setpoint = position;
    Ctrl.setReference(position, CANSparkMax.ControlType.kSmartMotion);

    /* NOTE:
     * This is equivalent to:
     * MotorDriver.set(speed);
     * 
     * But it has the advantage of canceling any PID control
     */
  }

  /**
   * Get the error for position control.
   */
  public double getPositionError() {
    return (getMotorPosition() - setpoint);
  }

  /**
   * Run the motor with a given speed.
   */
  public void spinMotor(double speed) {
    Ctrl.setReference(speed, CANSparkMax.ControlType.kDutyCycle);

    /* NOTE:
     * This is equivalent to:
     * MotorDriver.set(speed);
     * 
     * But it has the advantage of canceling any PID control
     */
  }

  /**
   * This method turns off power to the motor unconditionally.
   */
  public void motorOff() {
    MotorDriver.stopMotor();
  }

  /**
   * Update Shuffleboard with current data.
   */
  public void updateShuffleBoardValues(boolean WriteOnly) {
    shuf_velGoal.setDouble(0.0);

    // Motor subsection
    shuf_position.setDouble(getMotorPosition());
    shuf_velocity.setDouble(Encoder.getVelocity());

    // Readback values for tuning
    if (WriteOnly) {
      shuf_posGoal.setDouble(0.0);
      // For default values and tracking changes
      DebugP = Ctrl.getP();
      DebugI = Ctrl.getI();
      DebugD = Ctrl.getD();
      DebugIz = Ctrl.getIZone();
      DebugFF = Ctrl.getFF();
      DebugMinOut = Ctrl.getOutputMin();
      DebugMaxOut = Ctrl.getOutputMax();
      DebugMaxVel = Ctrl.getSmartMotionMaxVelocity(0);
      DebugMaxAcc = Ctrl.getSmartMotionMaxAccel(0);
      shuf_pidEnable.setInteger(0);
      shuf_pidP.setDouble(DebugP);
      shuf_pidI.setDouble(DebugI);
      shuf_pidD.setDouble(DebugD);
      shuf_pidIz.setDouble(DebugIz);
      shuf_pidFF.setDouble(DebugFF);
      shuf_pidMinOut.setDouble(DebugMinOut);
      shuf_pidMaxOut.setDouble(DebugMaxOut);
      shuf_maxVel.setDouble(DebugMaxVel);
      shuf_maxAcc.setDouble(DebugMaxAcc);
    } else {
      if (shuf_pidEnable.getInteger(0) == 1) {
        DebugP = shuf_pidP.getDouble(0);
        DebugI = shuf_pidI.getDouble(0);
        DebugD = shuf_pidD.getDouble(0);
        DebugIz = shuf_pidIz.getDouble(0);
        DebugFF = shuf_pidFF.getDouble(0);
        DebugMinOut = shuf_pidMinOut.getDouble(0);
        DebugMaxOut = shuf_pidMaxOut.getDouble(0);
        DebugMaxVel = shuf_maxVel.getDouble(0);
        DebugMaxAcc = shuf_maxAcc.getDouble(0);

        // if PID coefficients in Shuffleboard have changed, write new values to controller
        if((DebugP != kP)) { Ctrl.setP(DebugP); kP = DebugP; }
        if((DebugI != kI)) { Ctrl.setI(DebugI); kI = DebugI; }
        if((DebugD != kD)) { Ctrl.setD(DebugD); kD = DebugD; }
        if((DebugIz != kIz)) { Ctrl.setIZone(DebugIz); kIz = DebugIz; }
        if((DebugFF != kFF)) { Ctrl.setFF(DebugFF); kFF = DebugFF; }
        if((DebugMaxOut != kMaxOutput) || (DebugMinOut != kMinOutput)) { 
          Ctrl.setOutputRange(DebugMinOut, DebugMaxOut);
          kMinOutput = DebugMinOut; kMaxOutput = DebugMaxOut;
        }
        if((DebugMaxVel != maxVel)) { Ctrl.setSmartMotionMaxVelocity(maxVel, 0); }
        if((DebugMaxAcc != maxAcc)) { Ctrl.setSmartMotionMaxAccel(maxAcc, 0); }

        shuf_pidEnable.setInteger(0);
      }
    goal_pos = shuf_posGoal.getDouble(0);
    }


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Only show debug information when needed (not during a match!)
    if (Constants.kDebugLevel >= Constants.DEBUG_LEVEL_ALL) {
      updateShuffleBoardValues(false);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
