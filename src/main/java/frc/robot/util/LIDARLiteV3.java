package frc.robot.util;

import edu.wpi.first.wpilibj.Counter;

/**
 * This is a helper class for using a Garmin LIDAR-Lite v3
 */
public class LIDARLiteV3 {
  private Counter counter;

  private double _calibrationOffset = 0.0;
  private int _printedWarningCount = 5;
  
  /**
   * Create an instance of a LIDAR-Lite v3
   * 
   * @param port the roboRIO PWM port to which the LIDAR is attached
   * @param calibrationOffset the distance in cm from the front of the chassis to the LIDAR sensor
   */
  public LIDARLiteV3(int port, double calibrationOffset) {
    counter = new Counter(port); // plug the lidar into PWM <port>
    counter.setMaxPeriod(1.00); // set the max period that can be measured
    counter.setSemiPeriodMode(true); // Set the counter to period measurement
    counter.reset();

    _calibrationOffset = calibrationOffset;
  }

  /**
   * Used to get distance in cm
   * 
   * @param rounded true ? false: Get distance in inches as rounded number?
   * @return Distance in inches
   */
  public double getDistanceCm(boolean rounded) {
    double cm;
    /*
     * If we haven't seen the first rising to falling pulse, then we have no
     * measurement. This happens when there is no LIDAR-Lite plugged in, btw.
     */
    if (counter.get() < 1) {
      if (_printedWarningCount-- > 0) {
        System.out.println("LidarLitePWM: waiting for distance measurement");
      }
      return 0;
    }
    /*
     * getPeriod returns time in seconds. The hardware resolution is microseconds.
     * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of
     * distance.
     */
    cm = (counter.getPeriod() * 1000000.0 / 10.0) + _calibrationOffset;
    if (!rounded) {
      return cm;
    } else {
      return Math.floor(cm * 10) / 10;
    }
  }

  /**
   * Used to get distance in Inches
   * 
   * @param rounded true ? false: Get distance in inches as rounded number?
   * @return Distance in inches
   */
  public double getDistanceInches(boolean rounded) {
    double in = getDistanceCm(true) * 0.393700787;
    if (!rounded) {
      return in;
    } else {
      return Math.floor(in * 10) / 10;
    }
  }
}
