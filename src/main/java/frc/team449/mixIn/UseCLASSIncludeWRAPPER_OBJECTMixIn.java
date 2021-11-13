package frc.team449.mixIn;

import com.fasterxml.jackson.annotation.JsonTypeInfo;

/**
 * A mix-in that adds the annotation {@code @JsonTypeInfo(use=JsonTypeInfo.Id.CLASS,
 * include=JsonTypeInfo.As.WRAPPER_OBJECT)}. Don't make sublasses of this.
 */
@JsonTypeInfo(use = JsonTypeInfo.Id.CLASS, include = JsonTypeInfo.As.WRAPPER_OBJECT)
public interface UseCLASSIncludeWRAPPER_OBJECTMixIn {
}
