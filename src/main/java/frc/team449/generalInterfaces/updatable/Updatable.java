package frc.team449.generalInterfaces.updatable;

import com.fasterxml.jackson.annotation.JsonTypeInfo;

/**
 * An interface for any object that caches values. <br>
 * IMPORTANT: If you implement Updatable, be sure to call {@code Updater.register(this)} in your
 * constructor
 */
@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public interface Updatable {

  /** Updates all cached values with current ones. */
  void update();
}
