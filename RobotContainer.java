// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kIntake.kPivot.IntakePosition;
import frc.robot.commands.HandoffShoot;
import frc.robot.commands.IntakeCmds;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.HandoffRollers;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterFlywheels;

//import frc.robot.subsystems.SwerveSubsystem;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends TimedRobot {
  // The robot's subsystems and commands are defined here...
  
  public final CommandJoystick oopsieWoopsieController = new CommandJoystick(1);

  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  public final CommandXboxController driverController = new CommandXboxController(0);
  //private Command m_autonomousCommand;
  private final IntakePivot intakePivot = new IntakePivot();
  private final IntakeRollers intakeRollers = new IntakeRollers();
  private final HandoffRollers handoffRollers = new HandoffRollers();
  private final ShooterFlywheels flywheels = new ShooterFlywheels();
  //private final ShooterFlywheels flywheels = new ShooterFlywheels();
  public Boolean home = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //intakePivot.setDefaultCommand(intakePivot.setIntakePosition(IntakePosition.HOME));
    m_swerveSubsystem.setDefaultCommand(
        new SwerveJoystickCommand(
            m_swerveSubsystem,
            driverController,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> false  /*!driverController.y().getAsBoolean()*/  ));
    configureButtonBindings();
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    driverController.a().onTrue(
        runOnce(m_swerveSubsystem::zeroHeading));

    driverController.x().toggleOnTrue(
      XWheelConfigurationSetter());

    driverController.y().toggleOnTrue(
      intakeRollers.rawIntake());

// -=-=-=-=-=-

    driverController.leftBumper().onTrue(
      intakePivot.setIntakePosition(IntakePosition.HOME));
    
    // driverController.leftTrigger().onTrue(
    //   intakePivot.setIntakePosition(IntakePosition.DEPLOYED));

// -=-=-=-=-=-

    driverController.povDown().whileTrue(
      intakePivot.setIntakePosition(IntakePosition.EJECT));

      //new Trigger(() -> intakePivot.isAtPosition(IntakePosition.EJECT)).onTrue(intakeRollers.outtake().until(() -> intakeRollers.getPieceCheck().getAsBoolean() == false));

    driverController.rightTrigger().whileTrue(
      intakeRollers.outtake());

// -=-=-=-=-=-
    
    driverController.leftTrigger().onTrue(
        //Intake into robot and pivot
        new IntakeCmds(intakePivot, intakeRollers).intake()
        .andThen(new HandoffShoot(intakeRollers, handoffRollers).handOffNote()));

    driverController.rightBumper().toggleOnTrue(flywheels.shootVoltage(7));

        //Transfer Note to Shooter Body
        //new HandoffShoot(intakeRollers, handoffRollers).handOffNote(),
        //Prepare note for shooter flywheel by indexing to upper position
        //new HandoffShoot(intakeRollers, handoffRollers).indexToShooter()));

        /*
        new Trigger(() -> intakePivot.isAtPosition(IntakePosition.HOME))
           .and(intakeRollers.getPieceCheck())
           .and(() -> DriverStation.isTeleopEnabled())
           .onTrue(new HandoffShoot(intakeRollers, handoffRollers).handOffNote());
        */


    //oopsieWoopsieController.trigger().toggleOnTrue(flywheels.shootVoltage(5));

    }

    /*
    driverController.povLeft().onTrue(
      new HandoffShoot(intakeRollers, handoffRollers).handOffNote());
    */

//   -=-=-=-=- Specific Robot Operation Commands -=-=-=-=-

  public Command zeroHeading() {
    return m_swerveSubsystem.zeroHeadingCmd();
  }

  public Command XWheelConfigurationSetter() {
    return m_swerveSubsystem.setXWheelConfigurationCmd();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  //Not used
  public Command getAutonomousCommand() {
    // Will be run in autonomous
    return null;
  }
  
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    System.out.println("Teleop Periodic");
  }

}
