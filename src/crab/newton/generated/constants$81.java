// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$81 {

    static final FunctionDescriptor NewtonCorkscrewCalculateStopAccel$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonCorkscrewCalculateStopAccel$MH = RuntimeHelper.downcallHandle(
        "NewtonCorkscrewCalculateStopAccel",
        constants$81.NewtonCorkscrewCalculateStopAccel$FUNC, false
    );
    static final FunctionDescriptor NewtonConstraintCreateUniversal$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonConstraintCreateUniversal$MH = RuntimeHelper.downcallHandle(
        "NewtonConstraintCreateUniversal",
        constants$81.NewtonConstraintCreateUniversal$FUNC, false
    );
    static final FunctionDescriptor NewtonUniversalSetUserCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUniversalSetUserCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonUniversalSetUserCallback",
        constants$81.NewtonUniversalSetUserCallback$FUNC, false
    );
    static final FunctionDescriptor NewtonUniversalGetJointAngle0$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUniversalGetJointAngle0$MH = RuntimeHelper.downcallHandle(
        "NewtonUniversalGetJointAngle0",
        constants$81.NewtonUniversalGetJointAngle0$FUNC, false
    );
    static final FunctionDescriptor NewtonUniversalGetJointAngle1$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUniversalGetJointAngle1$MH = RuntimeHelper.downcallHandle(
        "NewtonUniversalGetJointAngle1",
        constants$81.NewtonUniversalGetJointAngle1$FUNC, false
    );
    static final FunctionDescriptor NewtonUniversalGetJointOmega0$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUniversalGetJointOmega0$MH = RuntimeHelper.downcallHandle(
        "NewtonUniversalGetJointOmega0",
        constants$81.NewtonUniversalGetJointOmega0$FUNC, false
    );
}

