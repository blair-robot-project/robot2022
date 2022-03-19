package frc.team449.oi;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.team449.updatable.Updatable;
import io.github.oblarg.oblog.Loggable;

@JsonTypeInfo(
    use = JsonTypeInfo.Id.CLASS,
    include = JsonTypeInfo.As.WRAPPER_OBJECT,
    property = "@class")
public interface OI extends Updatable, Loggable {}
