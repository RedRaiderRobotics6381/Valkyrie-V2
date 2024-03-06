package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    public CANSparkMax intakeMotor;
    public CANSparkMax indexerMotor;
    public CANSparkFlex launcherIndexerMotor;
    public static SparkPIDController intakePIDController;
    public boolean hasNote = false;
    //LauncherRotateSubsystem launcherRotateSubsystem = new LauncherRotateSubsystem();

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotor, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(Constants.IntakeConstants.kIndexerMotor, MotorType.kBrushless);
        launcherIndexerMotor = new CANSparkFlex(Constants.IntakeConstants.kLauncherIndexerMotor, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        intakeMotor.enableVoltageCompensation(12.0);
        intakeMotor.setSmartCurrentLimit(60);
        intakeMotor.setInverted(true);
        indexerMotor.setInverted(true);
        launcherIndexerMotor.setInverted(true);
        intakeMotor.burnFlash();  //Remove this after everything is up and running to save flash wear
    }

    @Override
    public void periodic() {

    }
  
    public void intakeIntake(){  
      if(Robot.sensorOuttake.get() == true){
        hasNote = true;
      } else {
        //new LauncherRotateCmd(LauncherConstants.posIntake, launcherRotateSubsystem);
        indexerMotor.set(IntakeConstants.indexerIntakeSpeed);
        intakeMotor.set(IntakeConstants.intakeSpeed);
        launcherIndexerMotor.set(IntakeConstants.launcherIndexerSpeed);
        //System.out.println(Robot.sensorIntake.get());
      }
    }
    
    public void intakeOuttake(){  
      if(Robot.sensorOuttake.get() == true){
        indexerMotor.set(IntakeConstants.indexerOuttakeSpeed);
        launcherIndexerMotor.set(IntakeConstants.indexerOuttakeSpeed);
      } else{
        hasNote = false;
      }
    }

    public void stopIntake(){  
      if(Robot.sensorOuttake.get() == true){
        hasNote = true;
      } else {
        //new LauncherRotateCmd(LauncherConstants.posIntake, launcherRotateSubsystem);
        indexerMotor.set(0);
        intakeMotor.set(0);
        launcherIndexerMotor.set(0);
        //System.out.println(Robot.sensorIntake.get());
      }
    }
    

}