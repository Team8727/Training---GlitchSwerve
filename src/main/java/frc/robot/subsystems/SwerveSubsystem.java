package frc.robot.subsystems;

import frc.robot.utilities.MAXSwerve;
import monologue.Annotations.Log;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.kModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
    
    
    //private ShuffleboardTab tab = Shuffleboard.getTab("Swerve Subsystem");

    private final MAXSwerve frontLeftModule =
        new MAXSwerve(
            kSwerve.CANID.frontLeftDrive, kSwerve.CANID.frontLeftSteer, kSwerve.Offsets.frontLeft);

    private final MAXSwerve backLeftModule =
        new MAXSwerve(
            kSwerve.CANID.backLeftDrive, kSwerve.CANID.backLeftSteer, kSwerve.Offsets.backLeft);

    private final MAXSwerve backRightModule =
        new MAXSwerve(
            kSwerve.CANID.backRightDrive, kSwerve.CANID.backRightSteer, kSwerve.Offsets.backRight);

    private final MAXSwerve frontRightModule =
        new MAXSwerve(
            kSwerve.CANID.frontRightDrive, kSwerve.CANID.frontRightSteer, kSwerve.Offsets.frontRight);


    private final AHRS navX = new AHRS(kSwerve.navxPort);
    private final SwerveDrivePoseEstimator poseEstimator; //TODO: Implement this

    public SwerveSubsystem(/*int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotor*/) {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        poseEstimator =
        new SwerveDrivePoseEstimator(
            kSwerve.kinematics,
            getGyroRaw(),
            getPositions(),
            new Pose2d(0, 0, new Rotation2d()),
            kSwerve.stateStdDevs,
            kSwerve.visionStdDevs);
    }

    public void zeroHeading() {
        navX.reset();
    }

    public Command zeroHeadingCmd() {
        return runOnce(navX::reset);
    }

    //This might be the getCorrectedSteer method in MAXSwerve
    public double getHeading() {
        return Math.IEEEremainder(navX.getAngle(), 360);
    }

    @Log.NT
    public Pose2d getPose() {
       return poseEstimator.getEstimatedPosition();
    }

    private Rotation2d getGyroRaw() {
        return navX.getRotation2d();
    }

      // Retrieve the positions (angle and distance traveled) for each swerve module
    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition(),
        frontRightModule.getPosition()
        };
    }

    //Get the steer angle of each swerve module in radians
    private double[] getActualSteerRadians() {
        return new double[] {
        frontLeftModule.getPosition().angle.getRadians(),
        backLeftModule.getPosition().angle.getRadians(),
        backRightModule.getPosition().angle.getRadians(),
        frontRightModule.getPosition().angle.getRadians()
        };
    }

    //Get the steer angle of each swerve module in degrees
    private double[] getActualSteerDegrees() {
        return new double[] {
        frontLeftModule.getPosition().angle.getDegrees(),
        backLeftModule.getPosition().angle.getDegrees(),
        backRightModule.getPosition().angle.getDegrees(),
        frontRightModule.getPosition().angle.getRadians()
        };
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());

        /*
        SmartDashboard.putNumberArray("Corrected Steer (frontLeft, frontRight, backLeft, backRight)", 
            new double[] {
                frontLeftModule.getCorrectedSteer().getRadians(), 
                frontRightModule.getCorrectedSteer().getRadians(), 
                backLeftModule.getCorrectedSteer().getRadians(),
                backRightModule.getCorrectedSteer().getRadians()
            });
        */

        /*
        SmartDashboard.putNumberArray("Steer Absolute Encoder (frontLeft, frontRight, backLeft, backRight)", 
            new double[] {
                frontLeftModule.getSteerEncoderAbsolute(), 
                frontRightModule.getSteerEncoderAbsolute(), 
                backLeftModule.getSteerEncoderAbsolute(),
                backRightModule.getSteerEncoderAbsolute()
            });
        */

        SmartDashboard.putNumberArray("Swerve Module Steer Angles (Radians)", this.getActualSteerRadians());
        SmartDashboard.putNumberArray("Swerve Module Steer Angles (Degrees)", this.getActualSteerDegrees());
    }

    public void setXWheelConfiguration() {
        frontLeftModule.setX();
        frontRightModule.setX();
        backLeftModule.setX();
        backRightModule.setX();
    }

    public Command setXWheelConfigurationCmd() {
        return run(() -> {
            frontLeftModule.setX();
            frontRightModule.setX();
            backLeftModule.setX();
            backRightModule.setX();
        });
    }

    public void stopModules() {
        //frontLeftModule.setBrakeMode(true);
        //frontRightModule.setBrakeMode(true);
        //backLeftModule.setBrakeMode(true);
        //backRightModule.setBrakeMode(true);

        //frontLeftModule.setRawDriveVoltage(0);
        //frontRightModule.setRawDriveVoltage(0);
        //backLeftModule.setRawDriveVoltage(0);
        //backRightModule.setRawDriveVoltage(0);

        //frontLeftModule.setX();
        //frontRightModule.setX();
        //backLeftModule.setX();
        //backRightModule.setX();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kModule.maxWheelSpeed);

        //TODO Test this (commented below) for each module instead of the current code (below), it might fix the random rotation reversing of the modules
        
        //Rotation2d FLCurrentAngle = new Rotation2d(frontLeftModule.getSteerEncoderAbsolute());
        frontLeftModule.setTargetState(desiredStates[0], true, true);
        //frontLeftModule.optimizedState.speedMetersPerSecond *= frontLeftModule.optimizedState.angle.minus(FLCurrentAngle).getCos();

        //Rotation2d FRCurrentAngle = new Rotation2d(frontRightModule.getSteerEncoderAbsolute());
        frontRightModule.setTargetState(desiredStates[1], true, true);
        //frontRightModule.optimizedState.speedMetersPerSecond *= frontRightModule.optimizedState.angle.minus(FRCurrentAngle).getCos();

        //Rotation2d BLCurrentAngle = new Rotation2d(backLeftModule.getSteerEncoderAbsolute());
        backLeftModule.setTargetState(desiredStates[2], true, true);
        //backLeftModule.optimizedState.speedMetersPerSecond *= backLeftModule.optimizedState.angle.minus(BLCurrentAngle).getCos();

        //Rotation2d BRCurrentAngle = new Rotation2d(backRightModule.getSteerEncoderAbsolute());
        backRightModule.setTargetState(desiredStates[3], true, true);
        //backRightModule.optimizedState.speedMetersPerSecond *= backRightModule.optimizedState.angle.minus(BRCurrentAngle).getCos();


        //frontLeftModule.setTargetState(desiredStates[0], true, true);
        //frontRightModule.setTargetState(desiredStates[1], true, true);

        //backLeftModule.setTargetState(desiredStates[2], true, true);
        //backRightModule.setTargetState(desiredStates[3], true, true);
    }
}
