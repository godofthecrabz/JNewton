// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$79 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$79() {}
    public static final FunctionDescriptor NewtonBodyGetType$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetType$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetType",
        constants$79.NewtonBodyGetType$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetCollidable$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetCollidable$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetCollidable",
        constants$79.NewtonBodyGetCollidable$FUNC
    );
    public static final FunctionDescriptor NewtonBodySetCollidable$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonBodySetCollidable$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetCollidable",
        constants$79.NewtonBodySetCollidable$FUNC
    );
    public static final FunctionDescriptor NewtonBodyAddForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyAddForce$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyAddForce",
        constants$79.NewtonBodyAddForce$FUNC
    );
    public static final FunctionDescriptor NewtonBodyAddTorque$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyAddTorque$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyAddTorque",
        constants$79.NewtonBodyAddTorque$FUNC
    );
    public static final FunctionDescriptor NewtonBodySetCentreOfMass$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodySetCentreOfMass$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetCentreOfMass",
        constants$79.NewtonBodySetCentreOfMass$FUNC
    );
}


