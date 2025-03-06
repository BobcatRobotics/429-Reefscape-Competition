// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final EightBitDo m_operatorController =
      new EightBitDo(Constants.Controllers.operator_controller_port);
  // Controller
  private final Ruffy leftRuffy = new Ruffy(0);
  private final Ruffy rightRuffy = new Ruffy(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Commands

  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final Climber m_climber = new Climber();

  public Command armUpCommand =
      new InstantCommand(() -> m_arm.runArm(Constants.ArmConstants.ARM_SPEED_UP));
  // public Command armHoldCommand = new InstantCommand(() -> m_arm.runArm(-.05));
  public Command armDownCommand =
      new InstantCommand(() -> m_arm.runArm(Constants.ArmConstants.ARM_SPEED_DOWN));
  public Command rollerInCommand =
      new InstantCommand(() -> m_roller.runRoller(Constants.RollerConstants.ROLLER_SPEED_IN));
  public Command rollerSlowOutCommand =
      new InstantCommand(() -> m_roller.runRoller(Constants.RollerConstants.ROLLER_SLOW_SPEED_OUT));
  public Command rollerFastOutCommand =
      new InstantCommand(() -> m_roller.runRoller(Constants.RollerConstants.ROLLER_FAST_SPEED_OUT));
  public Command climberWinchIn =
      new InstantCommand(() -> m_climber.runClimber(Constants.ClimberConstants.CLIMBER_SPEED_IN));
  public Command climberWinchOut =
      new InstantCommand(() -> m_climber.runClimber(Constants.ClimberConstants.CLIMBER_SPEED_OUT));

  public Command climberStopCommand = new InstantCommand(() -> m_climber.stopClimber());
  public Command armStopCommand = new InstantCommand(() -> m_arm.stopArm());
  public Command rollerStopCommand = new InstantCommand(() -> m_roller.stopRoller());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up namedCommands
    NamedCommands.registerCommand("outtake", rollerSlowOutCommand);
    NamedCommands.registerCommand("stopouttake", rollerStopCommand);
    NamedCommands.registerCommand("armout", armDownCommand);
    NamedCommands.registerCommand("armin", armUpCommand);
    NamedCommands.registerCommand("stoparm", armStopCommand);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    /*
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            CharacterizationCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            CharacterizationCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */
    autoChooser.addOption("Drive Forward", new PathPlannerAuto("driveforward"));
    autoChooser.addOption("Side", new PathPlannerAuto("side"));
    autoChooser.addOption("Slolam", new PathPlannerAuto("slolam"));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.JoystickDrive(
            drive,
            () -> -leftRuffy.getY(),
            () -> -leftRuffy.getX(),
            () -> -rightRuffy.getX(),
            leftRuffy.button));

    // Reset gyro to 0° when B button is pressed
    rightRuffy.button.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));

    m_operatorController.rb.whileTrue(armUpCommand).onFalse(armStopCommand);
    m_operatorController.lb.whileTrue(armDownCommand).onFalse(armStopCommand);

    m_operatorController.y.whileTrue(rollerInCommand).onFalse(rollerStopCommand);
    m_operatorController.b.whileTrue(rollerSlowOutCommand).onFalse(rollerStopCommand);
    m_operatorController.a.whileTrue(rollerFastOutCommand).onFalse(rollerStopCommand);

    m_operatorController.povUp.whileTrue(climberWinchIn).onFalse(climberStopCommand);
    m_operatorController.povDown.whileTrue(climberWinchOut).onFalse(climberStopCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
