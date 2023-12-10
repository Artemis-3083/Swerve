package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    
    WPI_CANCoder canCoder;
    WPI_TalonFX drive;
    WPI_TalonFX steer;  
  
  public SwerveModule(int canCoderID, int driveID, int steerID, double offset) {
    canCoder = new WPI_CANCoder(canCoderID);
    drive = new WPI_TalonFX(driveID);
    steer = new WPI_TalonFX(steerID);

    drive.configFactoryDefault();
    steer.configFactoryDefault();

    drive.config_kP(0, 1);
    drive.config_kI(0, 0);
    drive.config_kD(0, 0);
    drive.config_kF(0, 0);
    steer.config_kP(0, 1);
    steer.config_kI(0, 0);
    steer.config_kD(0, 0);
    steer.config_kF(0, 0);

    drive.configNominalOutputForward(0);
    drive.configNominalOutputReverse(0);
    steer.configNominalOutputForward(0);
    steer.configNominalOutputReverse(0);

    drive.configPeakOutputForward(1);
    drive.configPeakOutputReverse(-1);
    steer.configPeakOutputForward(1);
    steer.configPeakOutputReverse(-1);

    drive.setSelectedSensorPosition(0);
    steer.setSelectedSensorPosition(0);
    // steer.setSelectedSensorPosition((canCoder.getAbsolutePosition() - offset) / 360 * Constants.TALONFX_CPR);
  }

  public void stop(){
    drive.stopMotor();
    steer.stopMotor();
  }

  public void move(double drive, double rotation){
    this.drive.set(drive);
    steer.set(rotation);
  }

  public double getHeadingDegrees(){
    return steer.getSelectedSensorPosition() / Constants.TALONFX_CPR * Constants.DRIVE_GEAR_RATIO * 360;
  }

  public double steerSensoerPos(){
    return steer.getSelectedSensorPosition();
  }
  
  public double getVelocityRPM(){
    // return drive.getSelectedSensorVelocity() / Constants.DRIVE_GEAR_RATIO * 2 * Math.PI * Constants.DRIVE_WHEEL_RADIUS / 60;
    return drive.getSelectedSensorVelocity() * 600 / Constants.TALONFX_CPR * Constants.DRIVE_GEAR_RATIO;
  }

  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState optimizedDesiredState = optimize(desiredState, new Rotation2d(Math.toRadians(getHeadingDegrees())));
    double desiredSpeed = optimizedDesiredState.speedMetersPerSecond / Constants.DRIVE_GEAR_RATIO * 2 * Math.PI * Constants.DRIVE_WHEEL_RADIUS / 60;
    double desiredPosition = optimizedDesiredState.angle.getDegrees();
    SmartDashboard.putNumber("state speed", desiredSpeed);
    SmartDashboard.putNumber("state pos", desiredPosition);
    drive.set(ControlMode.Velocity, desiredSpeed);
    steer.set(ControlMode.Position, desiredPosition);
  }

  public double getAbsEncoder(){
    return canCoder.getAbsolutePosition();
  }

  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }
}
