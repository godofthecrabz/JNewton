// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$79 {

    static final FunctionDescriptor NewtonSliderGetJointPosit$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonSliderGetJointPosit$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderGetJointPosit",
        constants$79.NewtonSliderGetJointPosit$FUNC, false
    );
    static final FunctionDescriptor NewtonSliderGetJointVeloc$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonSliderGetJointVeloc$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderGetJointVeloc",
        constants$79.NewtonSliderGetJointVeloc$FUNC, false
    );
    static final FunctionDescriptor NewtonSliderGetJointForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonSliderGetJointForce$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderGetJointForce",
        constants$79.NewtonSliderGetJointForce$FUNC, false
    );
    static final FunctionDescriptor NewtonSliderCalculateStopAccel$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonSliderCalculateStopAccel$MH = RuntimeHelper.downcallHandle(
        "NewtonSliderCalculateStopAccel",
        constants$79.NewtonSliderCalculateStopAccel$FUNC, false
    );
    static final FunctionDescriptor NewtonConstraintCreateCorkscrew$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonConstraintCreateCorkscrew$MH = RuntimeHelper.downcallHandle(
        "NewtonConstraintCreateCorkscrew",
        constants$79.NewtonConstraintCreateCorkscrew$FUNC, false
    );
    static final FunctionDescriptor NewtonCorkscrewSetUserCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCorkscrewSetUserCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonCorkscrewSetUserCallback",
        constants$79.NewtonCorkscrewSetUserCallback$FUNC, false
    );
}

