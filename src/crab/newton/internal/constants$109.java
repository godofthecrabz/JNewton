// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$109 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$109() {}
    public static final FunctionDescriptor NewtonUserJointSetRowMaximumFriction$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonUserJointSetRowMaximumFriction$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointSetRowMaximumFriction",
        constants$109.NewtonUserJointSetRowMaximumFriction$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointCalculateRowZeroAcceleration$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserJointCalculateRowZeroAcceleration$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointCalculateRowZeroAcceleration",
        constants$109.NewtonUserJointCalculateRowZeroAcceleration$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointGetRowAcceleration$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserJointGetRowAcceleration$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointGetRowAcceleration",
        constants$109.NewtonUserJointGetRowAcceleration$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointGetRowJacobian$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserJointGetRowJacobian$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointGetRowJacobian",
        constants$109.NewtonUserJointGetRowJacobian$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointSetRowAcceleration$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonUserJointSetRowAcceleration$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointSetRowAcceleration",
        constants$109.NewtonUserJointSetRowAcceleration$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointSetRowMassDependentSpringDamperAcceleration$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonUserJointSetRowMassDependentSpringDamperAcceleration$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointSetRowMassDependentSpringDamperAcceleration",
        constants$109.NewtonUserJointSetRowMassDependentSpringDamperAcceleration$FUNC
    );
}


