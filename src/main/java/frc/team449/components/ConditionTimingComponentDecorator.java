package frc.team449.components;

import java.util.function.BooleanSupplier;

public class ConditionTimingComponentDecorator extends ConditionTimingComponent {
    private final BooleanSupplier source;

    public ConditionTimingComponentDecorator(
            final BooleanSupplier source, final boolean initialValue) {
        super(initialValue);
        this.source = source;
    }

    public void update(final double now) {
        super.update(now, this.source.getAsBoolean());
    }
}
