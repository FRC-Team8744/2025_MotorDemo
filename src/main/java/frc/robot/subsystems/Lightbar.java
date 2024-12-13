// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lightbar extends SubsystemBase {
  private final AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  // private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 120 LEDs per meter
  // private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  // private final LEDPattern m_scrollingRainbow =
  //     m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  /** Creates a new Lightbar. */
  public Lightbar() {
        // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    // m_ledBuffer.setRGB(1, 200, 200, 200);
    setRBG(5, 100, 0, 0);
    setRBG(6, 0, 100, 0);
    setRBG(7, 0, 0, 100);

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param r the r value [0-255]
   * @param g the g value [0-255]
   * @param b the b value [0-255]
   */
  public void setRBG(int index, int r, int b, int g) {
    m_ledBuffer.setRGB(index, r, b, g);
  }

  public void clearLEDs() {
    for (int index = 0; index < m_ledBuffer.getLength(); index++) {
      m_ledBuffer.setRGB(index, 0, 0, 0);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // Update the buffer with the rainbow animation
        // m_scrollingRainbow.applyTo(m_ledBuffer);
        // Set the LEDs
        m_led.setData(m_ledBuffer);
  }
}
