package frc.team449.updatable;

import com.fasterxml.jackson.annotation.JsonTypeInfo;

/** An interface for any object that caches values. */
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public interface Updatable {

  /** Updates all cached values with current ones. */
  void update();
}
