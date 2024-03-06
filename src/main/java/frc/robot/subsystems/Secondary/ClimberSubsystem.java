package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

//is this working?


public class ClimberSubsystem extends SubsystemBase{
    public CANSparkMax m_climberMotorL;  
    public CANSparkMax m_climberMotorR;
    public RelativeEncoder m_climberEncoderR;
    public RelativeEncoder m_climberEncoderL;
    public boolean climbed = false;
    public boolean lowered = false;
    private double climbDist = 12.25;
    private double lowerDist = 13.0;
    private double lowerStopDist = 0.5;

    /**
    * @param ClimberCmd
    */
    public ClimberSubsystem(){
        // Declare the motors
        m_climberMotorR = new CANSparkMax(ClimberConstants.kClimberMotorR, MotorType.kBrushless);
        m_climberMotorL = new CANSparkMax(ClimberConstants.kClimberMotorL, MotorType.kBrushless);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_climberMotorR.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        m_climberMotorL.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below        
        
        m_climberEncoderR = m_climberMotorR.getEncoder();
        m_climberEncoderL = m_climberMotorL.getEncoder();

        m_climberMotorL.setInverted(true);

        m_climberEncoderR.setPositionConversionFactor(.179); 
        m_climberEncoderL.setPositionConversionFactor(.179);

        m_climberMotorR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_climberMotorR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 14);
        m_climberMotorR.enableVoltageCompensation(12.0);
        m_climberMotorR.setSmartCurrentLimit(40);
        m_climberMotorR.setIdleMode(IdleMode.kBrake);

        m_climberMotorL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_climberMotorL.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 13);
        m_climberMotorL.enableVoltageCompensation(12.0);
        m_climberMotorL.setSmartCurrentLimit(40);
        m_climberMotorL.setIdleMode(IdleMode.kBrake);

        m_climberMotorR.burnFlash(); //Remove this after everything is up and running to save flash wear
        m_climberMotorL.burnFlash(); //Remove this after everything is up and running to save flash wear

    }

  @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("RClimber Enc Val", m_climberEncoderR.getPosition());
      SmartDashboard.putNumber("LClimber Enc Val", m_climberEncoderL.getPosition());
    }

    public void climberClimb() {
      if (m_climberEncoderR.getPosition() >= -0.1 &&
          m_climberEncoderR.getPosition() <= climbDist){
          m_climberMotorR.set(.75);
      }
      if (m_climberEncoderR.getPosition() >= climbDist) {
          m_climberMotorR.set(0);
      }
  
      if (m_climberEncoderL.getPosition() >= -0.1 &&
          m_climberEncoderL.getPosition() <= climbDist - 1.0){
          m_climberMotorL.set(.75);
        } 
      if (m_climberEncoderL.getPosition() >= climbDist - 1.0) {
          m_climberMotorL.set(0);
      }
      if (m_climberEncoderR.getPosition() >= climbDist &&
          m_climberEncoderL.getPosition() >= climbDist - 1.0){
          climbed = true;
      }
    }

    public void climberLower() {
      if (m_climberEncoderR.getPosition() >= lowerStopDist &&
        m_climberEncoderR.getPosition() <= lowerDist){
        m_climberMotorR.set(-.50);
      }
      if (m_climberEncoderR.getPosition() <= lowerStopDist) {
        m_climberMotorR.set(0);
      }

      if (m_climberEncoderL.getPosition() >= lowerStopDist &&
        m_climberEncoderL.getPosition() <= lowerDist){
        m_climberMotorL.set(-.50);
      } 
      if (m_climberEncoderL.getPosition() <= lowerStopDist) {
        m_climberMotorL.set(0);
      }
      if (m_climberEncoderR.getPosition() <= lowerStopDist &&
        m_climberEncoderL.getPosition() <= lowerStopDist){
        lowered = true;
      }
    }

    public void climberStop(){
      m_climberMotorR.set(0.0);
      m_climberMotorL.set(0.0);
    }

}