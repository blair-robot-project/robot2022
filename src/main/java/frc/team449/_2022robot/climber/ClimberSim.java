package frc.team449._2022robot.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.team449.wrappers.WrappedMotor;
import io.github.oblarg.oblog.annotations.Log;
import org.jetbrains.annotations.NotNull;

public class ClimberSim {
  public final double distanceTopBottom;
  private PivotingTelescopingClimber.ClimberState state;
  private final ElevatorSim elevatorSim;

  public ClimberSim(double distanceTopBottom) {
    this.distanceTopBottom = distanceTopBottom;
    // Start arm retracted
    this.state = PivotingTelescopingClimber.ClimberState.RETRACTED;
    this.elevatorSim = new ElevatorSim(DCMotor.getNEO(3), 1., 1., 1., 0., distanceTopBottom);
  }

  @Log.ToString
  public PivotingTelescopingClimber.ClimberState getState() {
    return state;
  }

  public void setState(PivotingTelescopingClimber.ClimberState state) {
    this.state = state;
  }

  //  public void pivotTelescopingArmOut() {
  //    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kForward);
  //  }
  //
  //  public void pivotTelescopingArmIn() {
  //    pivotingTelescopingArm.setSolenoid(DoubleSolenoid.Value.kReverse);
  //  }

  public void set(double velocity) {}

  public void reset() {}

  public void resetController() {}
}
