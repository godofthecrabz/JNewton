// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$85 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$85() {}
    public static final FunctionDescriptor NewtonBodyGetDestructorCallback$return$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonBodyGetDestructorCallback$return_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetDestructorCallback$return_UP$MH = RuntimeHelper.upcallHandle(NewtonBodyGetDestructorCallback$return.class, "apply", constants$85.NewtonBodyGetDestructorCallback$return_UP$FUNC);
    public static final FunctionDescriptor NewtonBodyGetDestructorCallback$return_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetDestructorCallback$return_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$85.NewtonBodyGetDestructorCallback$return_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetDestructorCallback$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetDestructorCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetDestructorCallback",
        constants$85.NewtonBodyGetDestructorCallback$FUNC
    );
    public static final FunctionDescriptor NewtonBodySetTransformCallback$callback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final FunctionDescriptor NewtonBodySetTransformCallback$callback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonBodySetTransformCallback$callback_UP$MH = RuntimeHelper.upcallHandle(NewtonBodySetTransformCallback$callback.class, "apply", constants$85.NewtonBodySetTransformCallback$callback_UP$FUNC);
    public static final FunctionDescriptor NewtonBodySetTransformCallback$callback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonBodySetTransformCallback$callback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$85.NewtonBodySetTransformCallback$callback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonBodySetTransformCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodySetTransformCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetTransformCallback",
        constants$85.NewtonBodySetTransformCallback$FUNC
    );
}


