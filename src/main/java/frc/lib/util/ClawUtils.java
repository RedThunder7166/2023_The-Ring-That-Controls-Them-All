package frc.lib.util;

public class ClawUtils {
    public static double degreesToEncoderUnits(double angle, double gearRatio) {
        double encoderUnitsPerMotorRotation = 2048;
        double encoderUnitsPerArmRotation = gearRatio * encoderUnitsPerMotorRotation;
    
        return angle * (encoderUnitsPerArmRotation / 360); //desired angle in degrees * encoder units per degree
    
      }
    
      public static double encoderUnitsToDegrees(double encoderUnits, double gearRatio) {
        // double gearRatio = 144;
        double encoderUnitsPerMotorRotation = 2048;
        double encoderUnitsPerArmRotation = gearRatio * encoderUnitsPerMotorRotation;
    
        return (encoderUnits * 360) / encoderUnitsPerArmRotation;
      }


      private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
    
      public static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }


}
