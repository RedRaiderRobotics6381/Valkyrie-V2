package frc.robot.subsystems.Secondary;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

    public CANSparkFlex m_launcherMotorTop;
    public CANSparkFlex m_launcherMotorBot;
    public SparkPIDController launcherPIDControllerTop;
    public static SparkPIDController launcherPIDControllerBot;
    public static RelativeEncoder encoderTop;
    public static RelativeEncoder m_encoderBot;
    public static double launcherSpeedSetpoint;
    public double currentLauncherSpeed;

    // double P = 0.0005;
    // double I = 0.0;
    // double D = 0.0;
  
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc;

    public LauncherSubsystem() {

        m_launcherMotorTop =  new CANSparkFlex(Constants.LauncherConstants.kLauncherT, MotorType.kBrushless);
        m_launcherMotorBot =  new CANSparkFlex(Constants.LauncherConstants.kLauncherB, MotorType.kBrushless);
        
        //m_encoderTop = m_launcherMotorTop.getAbsoluteEncoder();

        
        m_launcherMotorTop.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        m_launcherMotorBot.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        
        m_launcherMotorBot.follow(m_launcherMotorTop, true);

        // initialze PID controller and encoder objects
        launcherPIDControllerTop = m_launcherMotorTop.getPIDController();
        encoderTop = m_launcherMotorTop.getEncoder();
        // launcherPIDControllerBot = m_launcherMotorBot.getPIDController();
        //m_launcherMotorBot.setInverted(true);
        kP = 0.0006; 
        kI = 0.0000001;
        kD = 0.001; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 4250;
        maxAcc = 1500;

        m_launcherMotorTop.enableVoltageCompensation(12.0);
        m_launcherMotorTop.setSmartCurrentLimit(40);        

        
        launcherPIDControllerTop.setP(kP);
        launcherPIDControllerTop.setI(kI);
        launcherPIDControllerTop.setD(kD);
        launcherPIDControllerTop.setIZone(kIz);
        launcherPIDControllerTop.setFF(kFF);
        launcherPIDControllerTop.setOutputRange(kMinOutput, kMaxOutput);

        m_launcherMotorTop.burnFlash();  //Remove this after everything is up and running to save flash wear
        m_launcherMotorBot.burnFlash();  //Remove this after everything is up and running to save flash wear

        //launcherPIDControllerTop.setSmartMotionMaxVelocity(5000.0,0); //ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        //launcherPIDControllerTop.setSmartMotionMinOutputVelocity(0.0, 0); //ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        //launcherPIDControllerTop.setSmartMotionMaxAccel(1000.0,0); //ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
        
        int smartMotionSlot = 0;
        launcherPIDControllerTop.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        launcherPIDControllerTop.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        launcherPIDControllerTop.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        launcherPIDControllerTop.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher Set Speed", launcherSpeedSetpoint);
        SmartDashboard.putNumber("Launcher Raw Speed", encoderTop.getVelocity());
        SmartDashboard.putNumber("Launcher Rnd Speed", getLauncherSpeed());
        SmartDashboard.putBoolean("Launcher at Speed", speedComplete());
        //currentLauncherSpeed = (m_launcherMotorTop.getAbsoluteEncoder().getVelocity()) * 60;
        //launcherPIDControllerBot.setReference(1000, CANSparkFlex.ControlType.kSmartVelocity);
    }
    
    public Command LauncherCmd(double launcherSpeedSetpoint) {
        // implicitly require `this`
        return this.run(() -> launcherPIDControllerTop.setReference(launcherSpeedSetpoint, CANSparkFlex.ControlType.kVelocity));
    }
    
    public double getLauncherSpeed(){
        double launcherSpeed = (int)encoderTop.getVelocity();
        return launcherSpeed;
      }
  
      public boolean speedComplete(){
        if (getLauncherSpeed() == launcherSpeedSetpoint){
          return true;
        } else {
          return false;
        }
      }

}