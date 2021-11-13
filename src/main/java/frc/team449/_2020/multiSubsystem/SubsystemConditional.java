package frc.team449._2020.multiSubsystem;

import com.fasterxml.jackson.annotation.JsonTypeInfo;
import frc.team449.generalInterfaces.updatable.Updatable;
import io.github.oblarg.oblog.Loggable;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

/**
 * A subsystem with a condition that's sometimes met, e.g. a limit switch, a current/power limit, an
 * IR sensor.
 */
@JsonTypeInfo(
        use = JsonTypeInfo.Id.CLASS,
        include = JsonTypeInfo.As.WRAPPER_OBJECT,
        property = "@class")
public interface SubsystemConditional extends Updatable, Loggable {

    /**
     * Computes the current state of the condition.
     * @return {@code true} if the condition is met, {@code false} otherwise
     */
    boolean isConditionTrue();

    /**
     * Gets the state of the condition when {@link SubsystemConditional#update()} was last called.
     * @return {@code false} if the condition was met when cached, {@code false} otherwise
     */
    default boolean isConditionTrueCached() {
        return MixinImpl.isConditionTrueCached(this);
    }

    /**
     * Updates the cached value of the condition.
     */
    @Override
    default void update() {
        MixinImpl.update(this);
    }
}

@SuppressWarnings("ClassNameDiffersFromFileName")
final class MixinImpl {
    private static final ConcurrentMap<SubsystemConditional, Boolean> cachedConditions =
            new ConcurrentHashMap<>();

    private MixinImpl() {
    }

    static boolean isConditionTrueCached(final SubsystemConditional self) {
        final Boolean cached = cachedConditions.get(self);
        if (cached != null) return cached;

        final boolean updated = self.isConditionTrue();
        cachedConditions.put(self, updated);
        return updated;
    }

    static void update(final SubsystemConditional self) {
        cachedConditions.put(self, self.isConditionTrue());
    }
}
