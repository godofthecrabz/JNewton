// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$78 {

    static final FunctionDescriptor NewtonHingeGetJointAngle$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonHingeGetJointAngle$MH = RuntimeHelper.downcallHandle(
        "NewtonHingeGetJointAngle",
        constants$78.NewtonHingeGetJointAngle$FUNC, false
    );
    static final FunctionDescriptor NewtonHingeGetJointOmega$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonHingeGetJointOmega$MH = RuntimeHelper.downcallHandle(
        "NewtonHingeGetJointOmega",
        constants$78.NewtonHingeGetJointOmega$FUNC, false
    );
    static final FunctionDescriptor NewtonHingeGetJointForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonHingeGetJointForce$MH = RuntimeHelper.downcallHandle(
        "NewtonHingeGetJointForce",
        constants$78.NewtonHingeGetJointForce$FUNC, false
    );
    static final FunctionDescriptor NewtonHingeCalculateStopAlpha$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonHingeCalculateStopAlpha$MH = RuntimeHelper.downcallHandle(
        "NewtonHingeCalculateStopAlpha",
        constants$78.NewtonHingeCalculateStopAlpha$FUNC, false
    );
    static final FunctionDescriptor NewtonConstraintCreateSlider$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonConstraintCreateSlider$MH = RuntimeHelper.downcallHandle(
        "NewtonConstraintCreateSlider",
        constants$78.NewtonConstraintCreateSlider$FUNC, false
    );
    static final FunctionDescriptor NewtonSliderSetUserCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonSliderSetUserCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderSetUserCallback",
        constants$78.NewtonSliderSetUserCallback$FUNC, false
    );
}


