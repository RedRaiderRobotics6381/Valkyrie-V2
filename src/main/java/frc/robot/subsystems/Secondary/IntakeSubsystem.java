package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public CANSparkMax intakeMotor;
    public CANSparkMax indexerMotor;
    public CANSparkFlex launcherIndexerMotor;
    public static SparkPIDController intakePIDController;
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
    

}