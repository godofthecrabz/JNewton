// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$14 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$14() {}
    public static final FunctionDescriptor NewtonOnContactGeneration$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final FunctionDescriptor NewtonOnContactGeneration_UP$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonOnContactGeneration_UP$MH = RuntimeHelper.upcallHandle(NewtonOnContactGeneration.class, "apply", constants$14.NewtonOnContactGeneration_UP$FUNC);
    public static final FunctionDescriptor NewtonOnContactGeneration_DOWN$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonOnContactGeneration_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$14.NewtonOnContactGeneration_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonBodyIterator$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonBodyIterator_UP$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyIterator_UP$MH = RuntimeHelper.upcallHandle(NewtonBodyIterator.class, "apply", constants$14.NewtonBodyIterator_UP$FUNC);
    public static final FunctionDescriptor NewtonBodyIterator_DOWN$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyIterator_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$14.NewtonBodyIterator_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonJointIterator$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonJointIterator_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonJointIterator_UP$MH = RuntimeHelper.upcallHandle(NewtonJointIterator.class, "apply", constants$14.NewtonJointIterator_UP$FUNC);
    public static final FunctionDescriptor NewtonJointIterator_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonJointIterator_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$14.NewtonJointIterator_DOWN$FUNC
    );
}


