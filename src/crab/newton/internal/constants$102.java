// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$102 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$102() {}
    public static final FunctionDescriptor NewtonSliderGetJointPosit$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonSliderGetJointPosit$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderGetJointPosit",
        constants$102.NewtonSliderGetJointPosit$FUNC
    );
    public static final FunctionDescriptor NewtonSliderGetJointVeloc$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonSliderGetJointVeloc$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderGetJointVeloc",
        constants$102.NewtonSliderGetJointVeloc$FUNC
    );
    public static final FunctionDescriptor NewtonSliderGetJointForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonSliderGetJointForce$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderGetJointForce",
        constants$102.NewtonSliderGetJointForce$FUNC
    );
    public static final FunctionDescriptor NewtonSliderCalculateStopAccel$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonSliderCalculateStopAccel$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderCalculateStopAccel",
        constants$102.NewtonSliderCalculateStopAccel$FUNC
    );
    public static final FunctionDescriptor NewtonConstraintCreateCorkscrew$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonConstraintCreateCorkscrew$MH = RuntimeHelper.downcallHandle(
        "NewtonConstraintCreateCorkscrew",
        constants$102.NewtonConstraintCreateCorkscrew$FUNC
    );
    public static final FunctionDescriptor NewtonCorkscrewSetUserCallback$callback$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonCorkscrewSetUserCallback$callback_UP$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCorkscrewSetUserCallback$callback_UP$MH = RuntimeHelper.upcallHandle(NewtonCorkscrewSetUserCallback$callback.class, "apply", constants$102.NewtonCorkscrewSetUserCallback$callback_UP$FUNC);
}


