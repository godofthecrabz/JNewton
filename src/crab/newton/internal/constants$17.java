// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$17 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$17() {}
    public static final FunctionDescriptor NewtonUserBilateralCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final FunctionDescriptor NewtonUserBilateralCallback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonUserBilateralCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonUserBilateralCallback.class, "apply", constants$17.NewtonUserBilateralCallback_UP$FUNC);
    public static final FunctionDescriptor NewtonUserBilateralCallback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonUserBilateralCallback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$17.NewtonUserBilateralCallback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonUserBilateralGetInfoCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonUserBilateralGetInfoCallback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserBilateralGetInfoCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonUserBilateralGetInfoCallback.class, "apply", constants$17.NewtonUserBilateralGetInfoCallback_UP$FUNC);
    public static final FunctionDescriptor NewtonUserBilateralGetInfoCallback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserBilateralGetInfoCallback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$17.NewtonUserBilateralGetInfoCallback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonConstraintDestructor$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonConstraintDestructor_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonConstraintDestructor_UP$MH = RuntimeHelper.upcallHandle(NewtonConstraintDestructor.class, "apply", constants$17.NewtonConstraintDestructor_UP$FUNC);
    public static final FunctionDescriptor NewtonConstraintDestructor_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonConstraintDestructor_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$17.NewtonConstraintDestructor_DOWN$FUNC
    );
}


