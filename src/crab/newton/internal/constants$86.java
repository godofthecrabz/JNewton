// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$86 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$86() {}
    public static final FunctionDescriptor NewtonBodyGetTransformCallback$return$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final FunctionDescriptor NewtonBodyGetTransformCallback$return_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetTransformCallback$return_UP$MH = RuntimeHelper.upcallHandle(NewtonBodyGetTransformCallback$return.class, "apply", constants$86.NewtonBodyGetTransformCallback$return_UP$FUNC);
    public static final FunctionDescriptor NewtonBodyGetTransformCallback$return_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetTransformCallback$return_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$86.NewtonBodyGetTransformCallback$return_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetTransformCallback$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetTransformCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetTransformCallback",
        constants$86.NewtonBodyGetTransformCallback$FUNC
    );
    public static final FunctionDescriptor NewtonBodySetForceAndTorqueCallback$callback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final FunctionDescriptor NewtonBodySetForceAndTorqueCallback$callback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonBodySetForceAndTorqueCallback$callback_UP$MH = RuntimeHelper.upcallHandle(NewtonBodySetForceAndTorqueCallback$callback.class, "apply", constants$86.NewtonBodySetForceAndTorqueCallback$callback_UP$FUNC);
    public static final FunctionDescriptor NewtonBodySetForceAndTorqueCallback$callback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonBodySetForceAndTorqueCallback$callback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$86.NewtonBodySetForceAndTorqueCallback$callback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonBodySetForceAndTorqueCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodySetForceAndTorqueCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetForceAndTorqueCallback",
        constants$86.NewtonBodySetForceAndTorqueCallback$FUNC
    );
}


