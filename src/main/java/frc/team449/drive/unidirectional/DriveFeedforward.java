package frc.team449.drive.unidirectional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import frc.team449.other.Clock;
import io.github.oblarg.oblog.Loggable;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

/** To calculate feedforward for a drivetrain */
public interface DriveFeedforward extends Loggable {
  /**
   * Calculate feedforward/extra voltage for the left and right sides. This method may actually
   * mutate stuff, and it should be called every iteration of the control loop.
   *
   * @param leftVel The intended velocity of the left side
   * @param rightVel The intended velocity of the right side
   * @param leftAccel The intended acceleration on the left side
   * @param rightAccel The intended acceleration on the right side
   */
  @Contract(pure = false)
  @NotNull
  Pair<Double, Double> calculate(
      double leftVel, double rightVel, double leftAccel, double rightAccel);

  /** Update this feedforward object with the voltage given to the drivetrain it's controlling */
  default void updateVoltages(double leftVolt, double rightVolt) {}

  /** Uses {@link edu.wpi.first.math.controller.SimpleMotorFeedforward} to calculate feedforward */
  final class SimpleFF implements DriveFeedforward {
    @NotNull private final SimpleMotorFeedforward feedforward;

    public SimpleFF(double kS, double kV, double kA) {
      this.feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Contract("_, _, _, _ -> new")
    @NotNull
    @Override
    public Pair<Double, Double> calculate(
        double leftVel, double rightVel, double leftAccel, double rightAccel) {
      return Pair.of(
          feedforward.calculate(leftVel, leftAccel), feedforward.calculate(rightVel, rightAccel));
    }
  }

  /** Uses fancy linear algebra using WPI's state space API to calculate feedforward */
  final class LinearSystemFF implements DriveFeedforward {
    @NotNull private final LinearSystemLoop<N2, N2, N2> driveLoop;
    private double lastTime;

    public LinearSystemFF(@NotNull LinearSystemLoop<N2, N2, N2> driveLoop) {
      this.driveLoop = driveLoop;
      this.lastTime = Clock.currentTimeSeconds();
      this.updateVoltages(0, 0);
    }

    /**
     * Updates the loop with the new requested velocity and acceleration and calculate feedforward
     * to get there
     *
     * @param leftVel The intended velocity of the left side
     * @param rightVel The intended velocity of the right side
     * @param leftAccel The intended acceleration on the left side
     * @param rightAccel The intended acceleration on the right side
     */
    @Contract(pure = false)
    @NotNull
    @Override
    public Pair<Double, Double> calculate(
        double leftVel, double rightVel, double leftAccel, double rightAccel) {
      driveLoop.correct(VecBuilder.fill(leftVel, rightVel));

      var now = Clock.currentTimeSeconds();
      driveLoop.predict(now - lastTime);
      this.lastTime = now;

      // todo ensure that getU(1) is what we want
      return Pair.of(driveLoop.getU(0), driveLoop.getU(1));
    }

    @Override
    public void updateVoltages(double leftVolt, double rightVolt) {
      driveLoop.setNextR(VecBuilder.fill(leftVolt, rightVolt));
    }
  }
}
