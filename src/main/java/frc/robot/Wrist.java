package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ClawUtils;
import frc.robot.Constants.Clawstants;

public class Wrist extends SubsystemBase {
    private WPI_TalonFX m_wristMotor;
    private double wristAngleInEncoderUnits = 0;
    private double wristAngleInDegrees = 0;
    private CANCoder wristAbsolute;
    private static Wrist singleton;



    private Wrist() {
        m_wristMotor = new WPI_TalonFX(Clawstants.ClawMotorWristID);
        wristAbsolute = new CANCoder(Clawstants.wristEncoder);
        m_wristMotor.config_kP(0, 0.2);
        m_wristMotor.config_kI(0, 0);
        m_wristMotor.config_kD(0, .1);
        m_wristMotor.configClosedLoopPeakOutput(0, .2);

        m_wristMotor.setInverted(false);

        m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        m_wristMotor.setNeutralMode(NeutralMode.Brake);

        m_wristMotor.configMotionAcceleration(6000);
        m_wristMotor.configMotionCruiseVelocity(7000);

        wristAbsolute.configFactoryDefault();
        wristAbsolute.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        wristAbsolute.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        wristAbsolute.configMagnetOffset(-218); // WARNING: DO NOT CHANGE THIS UNLESS ENCODER HAS BEEN REMOVED AND PUT BACK ON
    }

    //Since we only have 1 arm, there is never a reason to have multiple arm objects. Thus, we use a singleton.
    public static Wrist getInstance(){
        if (singleton == null){
            singleton = new Wrist();
        }
        
        return singleton;
        
    }

    public void driveWrist(double speed){
        //WARNING: Gripper continues traveling 15-30 degrees past
        //max and min position.  Zero is set to where the gripper CANNOT
        //close anymore.  Max is set to 330 so it does not go past 360 which
        //loops back around to zero and breaks the follow logic
        double maxSpeed = .5;
        double maxClosed = 10;
        double maxOpen = 350;
        double position = wristAbsolute.getAbsolutePosition();
        boolean isMaxClosed = position <= maxClosed;
        boolean isMaxOpen = position >= maxOpen;
        boolean gripperClosing = speed > 0;
        boolean gripperOpening = speed < 0;
      
        //  if(isMaxClosed && gripperClosing){
        //   m_wristMotor.set(0);
        //  } else if(isMaxOpen && gripperOpening){
        //    m_wristMotor.set(0);
        //    } else {
        //      m_wristMotor.set(maxSpeed * speed);
        //      }
      m_wristMotor.set(speed);
    }

    public void setAngle(double wristAngle){
        wristAngleInDegrees = wristAngle;
        this.wristAngleInEncoderUnits = ClawUtils.degreesToEncoderUnits(wristAngle, Constants.Clawstants.wristGearRatio);

        m_wristMotor.set(
         ControlMode.MotionMagic,
         this.wristAngleInEncoderUnits,
         DemandType.ArbitraryFeedForward,
         Clawstants.wristFeedForward * java.lang.Math.cos(ClawUtils.encoderUnitsToDegrees(Math.toRadians(m_wristMotor.getSelectedSensorPosition()), Constants.Clawstants.wristGearRatio)));
    }

    public void zeroEncoder() {
        m_wristMotor.setSelectedSensorPosition(0);
        // m_armMotorRight.setSelectedSensorPosition(0);
      }

    public double getAngle(){
        return wristAngleInDegrees;
    }

    public double getEncoderUnits(){
        return wristAngleInEncoderUnits;
    }

    public double getRawEncoderUnits(){
        return m_wristMotor.getSelectedSensorPosition();
    }
}
