// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$36 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$36() {}
    public static final FunctionDescriptor NewtonWorldListenerSetPostUpdateCallback$callback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final FunctionDescriptor NewtonWorldListenerSetPostUpdateCallback$callback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetPostUpdateCallback$callback_UP$MH = RuntimeHelper.upcallHandle(NewtonWorldListenerSetPostUpdateCallback$callback.class, "apply", constants$36.NewtonWorldListenerSetPostUpdateCallback$callback_UP$FUNC);
    public static final FunctionDescriptor NewtonWorldListenerSetPostUpdateCallback$callback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetPostUpdateCallback$callback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$36.NewtonWorldListenerSetPostUpdateCallback$callback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonWorldListenerSetPostUpdateCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetPostUpdateCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerSetPostUpdateCallback",
        constants$36.NewtonWorldListenerSetPostUpdateCallback$FUNC
    );
    public static final FunctionDescriptor NewtonWorldListenerSetDestructorCallback$callback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonWorldListenerSetDestructorCallback$callback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetDestructorCallback$callback_UP$MH = RuntimeHelper.upcallHandle(NewtonWorldListenerSetDestructorCallback$callback.class, "apply", constants$36.NewtonWorldListenerSetDestructorCallback$callback_UP$FUNC);
    public static final FunctionDescriptor NewtonWorldListenerSetDestructorCallback$callback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetDestructorCallback$callback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$36.NewtonWorldListenerSetDestructorCallback$callback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonWorldListenerSetDestructorCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldListenerSetDestructorCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldListenerSetDestructorCallback",
        constants$36.NewtonWorldListenerSetDestructorCallback$FUNC
    );
}


