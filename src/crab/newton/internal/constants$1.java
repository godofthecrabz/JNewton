// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$1 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$1() {}
    public static final FunctionDescriptor NewtonPostUpdateCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final FunctionDescriptor NewtonPostUpdateCallback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonPostUpdateCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonPostUpdateCallback.class, "apply", constants$1.NewtonPostUpdateCallback_UP$FUNC);
    public static final FunctionDescriptor NewtonPostUpdateCallback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonPostUpdateCallback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$1.NewtonPostUpdateCallback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonCreateContactCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonCreateContactCallback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateContactCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonCreateContactCallback.class, "apply", constants$1.NewtonCreateContactCallback_UP$FUNC);
    public static final FunctionDescriptor NewtonCreateContactCallback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateContactCallback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$1.NewtonCreateContactCallback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonDestroyContactCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonDestroyContactCallback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonDestroyContactCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonDestroyContactCallback.class, "apply", constants$1.NewtonDestroyContactCallback_UP$FUNC);
    public static final FunctionDescriptor NewtonDestroyContactCallback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonDestroyContactCallback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$1.NewtonDestroyContactCallback_DOWN$FUNC
    );
}


