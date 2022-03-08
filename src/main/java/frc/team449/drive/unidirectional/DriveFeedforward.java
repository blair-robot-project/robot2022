package frc.team449.drive.unidirectional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import frc.team449.other.Clock;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

/** To calculate feedforward for a drivetrain */
public interface DriveFeedforward {
  /**
   * Calculate voltage for the left and right sides (the total voltage to set output to, not extra
   * voltage to add). This method may actually mutate stuff, and it should be called every iteration
   * of the control loop.
   *
   * @param leftVolt The intended voltage for the left side
   * @param rightVolt The intended voltage for the right side
   * @param leftVel The current velocity of the left side
   * @param rightVel The current velocity of the right side
   */
  @Contract(pure = false)
  @NotNull
  Pair<Double, Double> calculate(
      double leftVolt, double rightVolt, double leftVel, double rightVel);

  @NotNull
  SimpleMotorFeedforward asWpiFF();

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
        double leftVolt, double rightVolt, double leftVel, double rightVel) {
      return new Pair<>(
          leftVolt + feedforward.calculate(leftVolt), rightVolt + feedforward.calculate(rightVolt));
    }

    @NotNull
    @Override
    public SimpleMotorFeedforward asWpiFF() {
      return feedforward;
    }
  }

  /** Uses fancy linear algebra using WPI's state space API to calculate feedforward */
  final class LinearSystemFF implements DriveFeedforward {
    @NotNull private final LinearSystemLoop<N2, N2, N2> driveLoop;
    private double lastTime;

    public LinearSystemFF(@NotNull LinearSystemLoop<N2, N2, N2> driveLoop) {
      this.driveLoop = driveLoop;
      this.lastTime = Clock.currentTimeSeconds();
    }

    /**
     * Updates the loop and gets the next voltage needed
     *
     * @param leftVolt The intended voltage for the left side
     * @param rightVolt The intended voltage for the right side
     * @param leftVel The current velocity of the left side
     * @param rightVel The current velocity of the right side
     */
    @Contract(pure = false)
    @NotNull
    @Override
    public Pair<Double, Double> calculate(
        double leftVolt, double rightVolt, double leftVel, double rightVel) {
      driveLoop.setNextR(VecBuilder.fill(leftVolt, rightVolt));
      driveLoop.correct(VecBuilder.fill(leftVel, rightVel));

      var now = Clock.currentTimeSeconds();
      driveLoop.predict(now - lastTime);
      this.lastTime = now;

      // todo ensure that getU(1) is what we want
      return new Pair<>(driveLoop.getU(0), driveLoop.getU(1));
    }

    @NotNull
    @Override
    public SimpleMotorFeedforward asWpiFF() {
      // todo actually use kS, kV, and kA
      return new SimpleMotorFeedforward(0, 0, 0);
    }
  }
}
