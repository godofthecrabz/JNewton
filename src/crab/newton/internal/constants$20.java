// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$20 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$20() {}
    public static final FunctionDescriptor NewtonCreate$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT);
    public static final MethodHandle NewtonCreate$MH = RuntimeHelper.downcallHandleVariadic(
        "NewtonCreate",
        constants$20.NewtonCreate$FUNC
    );
    public static final FunctionDescriptor NewtonDestroy$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonDestroy$MH = RuntimeHelper.downcallHandle(
        "NewtonDestroy",
        constants$20.NewtonDestroy$FUNC
    );
    public static final FunctionDescriptor NewtonDestroyAllBodies$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonDestroyAllBodies$MH = RuntimeHelper.downcallHandle(
        "NewtonDestroyAllBodies",
        constants$20.NewtonDestroyAllBodies$FUNC
    );
    public static final FunctionDescriptor NewtonGetPostUpdateCallback$return$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final FunctionDescriptor NewtonGetPostUpdateCallback$return_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonGetPostUpdateCallback$return_UP$MH = RuntimeHelper.upcallHandle(NewtonGetPostUpdateCallback$return.class, "apply", constants$20.NewtonGetPostUpdateCallback$return_UP$FUNC);
    public static final FunctionDescriptor NewtonGetPostUpdateCallback$return_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonGetPostUpdateCallback$return_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$20.NewtonGetPostUpdateCallback$return_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonGetPostUpdateCallback$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonGetPostUpdateCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonGetPostUpdateCallback",
        constants$20.NewtonGetPostUpdateCallback$FUNC
    );
}


