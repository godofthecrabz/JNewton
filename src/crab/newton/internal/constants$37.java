// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$37 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$37() {}
    public static final FunctionDescriptor NewtonWorldListenerSetBodyDestroyCallback$callback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonWorldListenerSetBodyDestroyCallback$callback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetBodyDestroyCallback$callback_UP$MH = RuntimeHelper.upcallHandle(NewtonWorldListenerSetBodyDestroyCallback$callback.class, "apply", constants$37.NewtonWorldListenerSetBodyDestroyCallback$callback_UP$FUNC);
    public static final FunctionDescriptor NewtonWorldListenerSetBodyDestroyCallback$callback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetBodyDestroyCallback$callback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$37.NewtonWorldListenerSetBodyDestroyCallback$callback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonWorldListenerSetBodyDestroyCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetBodyDestroyCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerSetBodyDestroyCallback",
        constants$37.NewtonWorldListenerSetBodyDestroyCallback$FUNC
    );
    public static final FunctionDescriptor NewtonWorldListenerDebug$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerDebug$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerDebug",
        constants$37.NewtonWorldListenerDebug$FUNC
    );
    public static final FunctionDescriptor NewtonWorldGetListenerUserData$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldGetListenerUserData$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldGetListenerUserData",
        constants$37.NewtonWorldGetListenerUserData$FUNC
    );
    public static final FunctionDescriptor NewtonWorldListenerGetBodyDestroyCallback$return$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonWorldListenerGetBodyDestroyCallback$return_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerGetBodyDestroyCallback$return_UP$MH = RuntimeHelper.upcallHandle(NewtonWorldListenerGetBodyDestroyCallback$return.class, "apply", constants$37.NewtonWorldListenerGetBodyDestroyCallback$return_UP$FUNC);
}


