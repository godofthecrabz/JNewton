// Generated by jextract
package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public class constants$68 {

    public static final FunctionDescriptor NewtonBodyGetInvMass$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetInvMass$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetInvMass",
        constants$68.NewtonBodyGetInvMass$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetInertiaMatrix$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetInertiaMatrix$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetInertiaMatrix",
        constants$68.NewtonBodyGetInertiaMatrix$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetInvInertiaMatrix$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetInvInertiaMatrix$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetInvInertiaMatrix",
        constants$68.NewtonBodyGetInvInertiaMatrix$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetOmega$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetOmega$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetOmega",
        constants$68.NewtonBodyGetOmega$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetVelocity$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetVelocity$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetVelocity",
        constants$68.NewtonBodyGetVelocity$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetAlpha$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetAlpha$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetAlpha",
        constants$68.NewtonBodyGetAlpha$FUNC
    );
}


