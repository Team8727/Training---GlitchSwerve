// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.kShooter.kHandoffRollers;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class HandoffRollers extends SubsystemBase {
  /** Creates a new HandoffRollers. */
  private TalonSRX handoffMotor = new TalonSRX(kHandoffRollers.canID);
  private final DigitalInput upperSensor = new DigitalInput(kHandoffRollers.upperSensorPort);
  private final DigitalInput lowerSensor = new DigitalInput(kHandoffRollers.lowerSensorPort);

  public HandoffRollers() {
    handoffMotor.setInverted(kHandoffRollers.inverted);
    handoffMotor.setNeutralMode(NeutralMode.Coast);

    Shuffleboard.getTab("Driver Info").addBoolean("Upper Sensor", () -> getUpperSensor());
    //Shuffleboard.getTab("Driver Info").addBoolean("Has Piece", this::hasPiece);

  }

  public void setVoltage(double voltage) {
    handoffMotor.set(ControlMode.PercentOutput, voltage / 12.0);
  }

  public boolean getUpperSensor() {
    return !upperSensor.get();
  }

  public boolean getLowerSensor() {
    return lowerSensor.get();
  }

  public boolean hasPiece() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command handoffIndexIntake() {
    return run(
      () -> setVoltage(kHandoffRollers.intakeVoltage))
    .until(() -> getUpperSensor()).withTimeout(3)
    .finallyDo(() -> setVoltage(0));
  }

  public Command handoffShooter() {
    return run(
      () -> setVoltage(kHandoffRollers.shooterFeedVoltage))
    .until(() -> !getUpperSensor())
    .andThen(() -> setVoltage(0));
      
  }

  public Command feedToShoot() {
    return runEnd(
      () -> setVoltage(kHandoffRollers.shooterFeedVoltage),
      () -> setVoltage(0));
  }
}
