package frc.team449.drive.shifting;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.team449.generalInterfaces.shiftable.Shiftable;

/** A drive that has a high gear and a low gear and can switch between them. */
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public interface DriveShiftable extends Shiftable {

  /** @return true if currently overriding autoshifting, false otherwise. */
  @SuppressWarnings("BooleanMethodIsAlwaysInverted")
  boolean getOverrideAutoshift();

  /** @param override Whether or not to override autoshifting. */
  void setOverrideAutoshift(boolean override);
}
