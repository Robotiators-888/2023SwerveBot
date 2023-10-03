package frc.robot.Commands;


import frc.robot.Subsystems.SUB_Drivetrain;
import frc.robot.Subsystems.SUB_Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CMD_limeAlign extends CommandBase {
  SUB_Limelight limelight;
  SUB_Drivetrain drivetrain;
  
  /**
   * turn to goal using limelight
   * @param limeIn
   * @param driveIn
   */
  public CMD_limeAlign(SUB_Limelight limeIn, SUB_Drivetrain driveIn) {
    this.limelight = limeIn;
    this.drivetrain = driveIn;
    addRequirements(limeIn, driveIn);
  }

  @Override
  public void initialize() {
    limelight.setLed(3);
  }

  @Override
  public void execute() {
    if ((Math.abs(limelight.getTx()) > 5)) {
      drivetrain.drive(0, Math.signum(limelight.getTx()) * 0.3, 0., true, true);
    } else {
      // break the motors
      drivetrain.drive(0, 0, 0, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((limelight.getTv() && (limelight.getTx() < 4) && (limelight.getTx() > -4))) {
      return true;
    } else {
      return false;
    }
  }
}