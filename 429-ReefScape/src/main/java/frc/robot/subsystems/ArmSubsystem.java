package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX armMotor;
  //private final DigitalInput armSwitchPressed;

  /** This subsytem that controls the arm. */
  public ArmSubsystem() {
    // Set up a new digital input (for the limit switch)
    //armSwitchPressed = new DigitalInput(ArmConstants.ARM_SWITCH_PORT);
    // Set up the arm motor as a brushed motor
    armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //    config.CurrentLimits.withSupplyCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT); dont do
    // this, it doesnt actually assign the value to the config
    // use this instead
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.ARM_MOTOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ArmConstants.ARM_MOTOR_STATOR_CURRENT_LIMIT;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25; // TODO tune this
    armMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {}

  /**
   * This is a method that makes the arm move at your desired speed Positive values make it spin
   * forward and negative values spin it in reverse
   *
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runArm(double speed) {
    // if the limit switch is pressed, stop the arm motor
    // if (!armSwitchPressed.get()) {
    //  armMotor.stopMotor();
    // } else {
    armMotor.set(speed);
    // }
  }

  public void stopArm() {
    armMotor.stopMotor();
  }
}
