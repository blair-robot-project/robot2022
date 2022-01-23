package frc.team449.javaMaps.builders;

import edu.wpi.first.wpilibj.GenericHID;
import frc.team449.generalInterfaces.doubleUnaryOperator.Polynomial;
import frc.team449.oi.throttles.ThrottlePolynomial;
import org.jetbrains.annotations.NotNull;

public class ThrottlePolynomialBuilder {

  private GenericHID stick;
  private int axis;
  private double deadband;
  private Double smoothingTimeSecs;
  private boolean inverted;
  private Polynomial polynomial;
  private Double scale;

  public ThrottlePolynomialBuilder stick(@NotNull GenericHID stick) {
    this.stick = stick;
    return this;
  }

  public ThrottlePolynomialBuilder axis(int axis) {
    this.axis = axis;
    return this;
  }

  public ThrottlePolynomialBuilder deadband(double deadband) {
    this.deadband = deadband;
    return this;
  }

  public ThrottlePolynomialBuilder smoothingTimeSecs(double smoothingTimeSecs) {
    this.smoothingTimeSecs = smoothingTimeSecs;
    return this;
  }

  public ThrottlePolynomialBuilder inverted(boolean inverted) {
    this.inverted = inverted;
    return this;
  }

  public ThrottlePolynomialBuilder polynomial(@NotNull Polynomial polynomial) {
    this.polynomial = polynomial;
    return this;
  }

  public ThrottlePolynomialBuilder scale(double scale) {
    this.scale = scale;
    return this;
  }

  public ThrottlePolynomial build() {
    assert stick != null : "Stick for ThrottlePolynomial must not be null";
    assert polynomial != null : "Polynomial for ThrottlePolynomial must not be null";
    return new ThrottlePolynomial(
        stick, axis, deadband, smoothingTimeSecs, inverted, polynomial, scale);
  }
}
