// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$82 {

    static final FunctionDescriptor NewtonUniversalGetJointOmega1$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUniversalGetJointOmega1$MH = RuntimeHelper.downcallHandle(
        "NewtonUniversalGetJointOmega1",
        constants$82.NewtonUniversalGetJointOmega1$FUNC, false
    );
    static final FunctionDescriptor NewtonUniversalGetJointForce$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUniversalGetJointForce$MH = RuntimeHelper.downcallHandle(
        "NewtonUniversalGetJointForce",
        constants$82.NewtonUniversalGetJointForce$FUNC, false
    );
    static final FunctionDescriptor NewtonUniversalCalculateStopAlpha0$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonUniversalCalculateStopAlpha0$MH = RuntimeHelper.downcallHandle(
        "NewtonUniversalCalculateStopAlpha0",
        constants$82.NewtonUniversalCalculateStopAlpha0$FUNC, false
    );
    static final FunctionDescriptor NewtonUniversalCalculateStopAlpha1$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonUniversalCalculateStopAlpha1$MH = RuntimeHelper.downcallHandle(
        "NewtonUniversalCalculateStopAlpha1",
        constants$82.NewtonUniversalCalculateStopAlpha1$FUNC, false
    );
    static final FunctionDescriptor NewtonConstraintCreateUpVector$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonConstraintCreateUpVector$MH = RuntimeHelper.downcallHandle(
        "NewtonConstraintCreateUpVector",
        constants$82.NewtonConstraintCreateUpVector$FUNC, false
    );
    static final FunctionDescriptor NewtonUpVectorGetPin$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonUpVectorGetPin$MH = RuntimeHelper.downcallHandle(
        "NewtonUpVectorGetPin",
        constants$82.NewtonUpVectorGetPin$FUNC, false
    );
}

