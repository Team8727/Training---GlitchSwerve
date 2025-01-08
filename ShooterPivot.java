// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {
  /** Creates a new ShooterPivot. */
  public ShooterPivot() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*
  public double calculateVoltage(Rotation2d angle) {
    // Get setpoint from profile
    var nextSetpoint = pivotController.getSetpoint();

    var accel = (nextSetpoint.velocity - currentSetpoint.velocity) / 0.02;

    // Calculate voltages
    double feedForwardVoltage =
        pivotFF.calculate(
            nextSetpoint.position + kPivot.cogOffset.getRadians(), nextSetpoint.velocity, accel);
    double feedbackVoltage = pivotController.calculate(getPivotAngle().getRadians());

    // Log Values
    this.log("FeedbackVoltage", feedbackVoltage);
    this.log("Feedforward voltage", feedForwardVoltage);

    currentSetpoint.position = nextSetpoint.position;
    currentSetpoint.velocity = nextSetpoint.velocity;

    return feedForwardVoltage + feedbackVoltage;
  }
  */
}
