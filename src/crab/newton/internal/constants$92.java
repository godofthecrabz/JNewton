// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$92 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$92() {}
    public static final FunctionDescriptor NewtonBodyIntegrateVelocity$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonBodyIntegrateVelocity$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyIntegrateVelocity",
        constants$92.NewtonBodyIntegrateVelocity$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetLinearDamping$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetLinearDamping$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetLinearDamping",
        constants$92.NewtonBodyGetLinearDamping$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetAngularDamping$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetAngularDamping$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetAngularDamping",
        constants$92.NewtonBodyGetAngularDamping$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetAABB$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetAABB$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetAABB",
        constants$92.NewtonBodyGetAABB$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetFirstJoint$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetFirstJoint$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetFirstJoint",
        constants$92.NewtonBodyGetFirstJoint$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetNextJoint$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetNextJoint$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetNextJoint",
        constants$92.NewtonBodyGetNextJoint$FUNC
    );
}


