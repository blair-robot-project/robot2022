import frc.team449.Robot;
import org.junit.Assert;
import org.junit.Test;

public final class Tests {
  @org.junit.Before
  public void before() {
    Robot.notifyTesting();
  }

  @Test
  public void deserializeMap() {
    Assert.assertNotNull(Robot.loadMap());
    System.out.println(
        "*******************************************\n"
            + "MAP DESERIALIZATION SUCCESSFUL\n"
            + "*******************************************");
  }
}
