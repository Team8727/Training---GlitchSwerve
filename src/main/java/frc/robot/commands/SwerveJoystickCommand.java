// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kOI;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import java.util.function.Supplier;

public class SwerveJoystickCommand extends Command {
  /** Creates a new SwerveJoystickCommand. */
  private final SwerveSubsystem m_swerveSubsystem;
  private final Supplier<Double> m_xSpdFunction, m_ySpdFunction, m_zRotationFunction;
  private final Supplier<Boolean> m_fieldOrientedFunction;
  private final CommandXboxController m_driverController;
  //private final SlewRateLimiter m_xLimiter, m_yLimiter, m_turningLimiter;
  private PhotonCamera camera1 = new PhotonCamera("camera1");
  //private PhotonCamera camera2 = new PhotonCamera("camera2");
  //private RobotContainer m_RobotContainer = new RobotContainer();

  public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem, CommandXboxController driverController, 
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
    Supplier<Double> zRotationFunction, Supplier<Boolean> fieldOrientedFunction) {

      m_swerveSubsystem = swerveSubsystem;
      m_xSpdFunction = xSpdFunction;
      m_ySpdFunction = ySpdFunction;
      m_zRotationFunction = zRotationFunction;
      m_fieldOrientedFunction = fieldOrientedFunction;
      m_driverController = driverController;

      //m_xLimiter = new SlewRateLimiter(kSwerve.maxTransAccel);
      //m_yLimiter = new SlewRateLimiter(kSwerve.maxTransAccel);
      //m_turningLimiter = new SlewRateLimiter(kSwerve.maxAngAccel);

      addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. Get joystick inputs
    double xSpeed = m_xSpdFunction.get();
    double ySpeed = m_ySpdFunction.get();
    double zRotation = m_zRotationFunction.get();

    // 2. Apply deadband (protects against sudden joystick movement)
    xSpeed = MathUtil.applyDeadband(xSpeed, kOI.translationDeadzone); //Math.abs(xSpeed) > SwerveJoystick.kDeadband ? xSpeed : 0.0;
    ySpeed = MathUtil.applyDeadband(ySpeed, kOI.translationDeadzone); //Math.abs(ySpeed) > SwerveJoystick.kDeadband ? ySpeed : 0.0;
    zRotation = MathUtil.applyDeadband(zRotation, kOI.rotationDeadzone); //Math.abs(turningSpeed) > SwerveJoystick.kDeadband ? turningSpeed : 0.0;

    // 3. Make the driving smoother
    xSpeed *= kSwerve.maxTransSpeed; //m_xLimiter.calculate(xSpeed) * kSwerve.maxTransSpeed;
    ySpeed *= kSwerve.maxTransSpeed; //m_yLimiter.calculate(ySpeed) * kSwerve.maxTransSpeed;
    zRotation *= kSwerve.maxAngSpeed; //m_turningLimiter.calculate(turningSpeed) * kSwerve.maxAngSpeed;

    boolean targetVisible = false;
    double targetYaw = 0.0;
    var result = camera1.getLatestResult();

    
    //-=-=-=-=-=-=-=- Vision Code -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    for (var target : result.getTargets()) { //                                   |
      System.out.println(target.getFiducialId()); //                            |
            if (target.getFiducialId() == 9) { //                                 |
                // Found Tag 9, record its information                            |
                targetYaw = target.getYaw(); //                                   |
                targetVisible = true; //                                          |
            } //                                                                  |
        } //                                                                      |
        //                                                                        |
    if (targetVisible && m_driverController.povRight().getAsBoolean()) { //  |
        // Driver wants auto-alignment to tag #                                   |
        // And, tag # is in sight, so we can turn toward it.                      |
        // Override the driver's turn command with an automatic                   |
        // one that turns toward the tag.                                         |
        zRotation = -1.0 * targetYaw; //                                          |
      } //                                                                        |
      
    //-=-=-=-=-=-=-=- End of Vision Code -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (m_fieldOrientedFunction.get()) {
      //Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zRotation, m_swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = kSwerve.kinematics.toSwerveModuleStates(chassisSpeeds);
    //System.out.println("Here is all moduleStates: " + moduleStates);
    //System.out.println("Here is moduleStates[0]: " + moduleStates[0]);
    //System.out.println("Here is moduleStates[1]: " + moduleStates[1]);
    //System.out.println("Here is moduleStates[2]: " + moduleStates[2]);
    //System.out.println("Here is moduleStates[3]: " + moduleStates[3]);

    // 6. Output each module states to wheels
    m_swerveSubsystem.setModuleStates(moduleStates);
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stopModules(); // This does nothing right now as the method is empty, the motors were set already to an idle mode of kBrake.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
