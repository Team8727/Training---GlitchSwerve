// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.kIntakeShooter.kShootSpeaker;
import frc.robot.Constants.kShooter.kFlywheels;
import frc.robot.Constants.kShooter.kFlywheels.kFlywheel2;
import frc.robot.Constants.kShooter.kFlywheels.kFlywheel1;
import frc.robot.utilities.SparkConfigurator.LogData;

import java.util.Set;

public class ShooterFlywheels extends SubsystemBase {
  /** Creates a new ShooterFlywheels. */
  
  CANSparkMax flywheel1; // = new CANSparkMax(kFlywheels.kFlywheel1.canID, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax flywheel2; // = new CANSparkMax(kFlywheels.kFlywheel2.canID, CANSparkLowLevel.MotorType.kBrushless);
  private final SimpleMotorFeedforward fly1FF;
  private final SimpleMotorFeedforward fly2FF;
  private final SparkPIDController fly1PID;
  private final SparkPIDController fly2PID;
  private final RelativeEncoder fly1Encoder;
  private final RelativeEncoder fly2Encoder;
  private double setpoint;
  private boolean hasPiece;

  public ShooterFlywheels() {
    flywheel1 = getSparkMax(kFlywheels.kFlywheel1.canID, CANSparkLowLevel.MotorType.kBrushless, false, Set.of(), Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE, LogData.CURRENT));
    flywheel2 = getSparkMax(kFlywheels.kFlywheel2.canID, CANSparkLowLevel.MotorType.kBrushless, false, Set.of(), Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE, LogData.CURRENT));
  
    flywheel1.setInverted(kFlywheels.invert);
    flywheel2.setInverted(!kFlywheels.invert);
    flywheel1.setIdleMode(IdleMode.kBrake);
    flywheel2.setIdleMode(IdleMode.kBrake);

    // FeedForwards
    fly1FF = new SimpleMotorFeedforward(kFlywheel1.ks, kFlywheel1.kv, kFlywheel1.ka);
    fly2FF = new SimpleMotorFeedforward(kFlywheel2.ks, kFlywheel2.kv, kFlywheel2.ka);

    // Encoders
    fly1Encoder = flywheel1.getEncoder();
    fly1Encoder.setPositionConversionFactor(kFlywheels.positionConversionFactor);
    fly1Encoder.setVelocityConversionFactor(kFlywheels.velocityConversionFactor);
    fly1Encoder.setAverageDepth(4);
    fly1Encoder.setMeasurementPeriod(12);

    fly2Encoder = flywheel2.getEncoder();
    fly2Encoder.setPositionConversionFactor(kFlywheels.positionConversionFactor);
    fly2Encoder.setVelocityConversionFactor(kFlywheels.velocityConversionFactor);
    fly2Encoder.setAverageDepth(4);
    fly2Encoder.setMeasurementPeriod(12);

    fly1PID = flywheel1.getPIDController();
    fly1PID.setFeedbackDevice(fly1Encoder);
    fly1PID.setOutputRange(kFlywheel1.minPIDOutput, kFlywheel1.maxPIDOutput);
    fly1PID.setP(kFlywheel1.kP);
    fly1PID.setD(kFlywheel1.kD);


    fly2PID = flywheel2.getPIDController();
    fly2PID.setFeedbackDevice(fly2Encoder);
    fly2PID.setOutputRange(kFlywheel2.minPIDOutput, kFlywheel2.maxPIDOutput);
    fly2PID.setP(kFlywheel2.kP);
    fly2PID.setD(kFlywheel2.kD);
    }

  public void setVoltage(double voltage) {
    flywheel1.setVoltage(voltage);
    flywheel2.setVoltage(voltage);
  }

  public void setVelocity(double velocity) {
    setpoint = velocity;
    fly1PID.setReference(velocity, ControlType.kVelocity, 0, fly1FF.calculate(velocity));
    fly2PID.setReference(velocity, ControlType.kVelocity, 0, fly2FF.calculate(velocity));
  }

  public Command setShooterSpeed(double velocity) {
    return run(() -> setVelocity(velocity)).finallyDo(() -> setVoltage(0));
  }

  public Command shootVoltage(double voltage) {
    return startEnd(
      () -> setVoltage(voltage), 
      () -> setVoltage(0));
  }

  public Command spinUpSpeaker() {
    return this.run(() -> setVoltage(kShootSpeaker.shootVoltage));
  }

  public Command shootSpeaker() {
    return shootVoltage(kShootSpeaker.shootVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
