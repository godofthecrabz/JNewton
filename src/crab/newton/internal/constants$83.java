// Generated by jextract
package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public class constants$83 {

    public static final FunctionDescriptor NewtonUpVectorSetPin$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUpVectorSetPin$MH = RuntimeHelper.downcallHandle(
        "NewtonUpVectorSetPin",
        constants$83.NewtonUpVectorSetPin$FUNC
    );
    public static final FunctionDescriptor NewtonConstraintCreateUserJoint$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonConstraintCreateUserJoint$MH = RuntimeHelper.downcallHandle(
        "NewtonConstraintCreateUserJoint",
        constants$83.NewtonConstraintCreateUserJoint$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointGetSolverModel$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserJointGetSolverModel$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointGetSolverModel",
        constants$83.NewtonUserJointGetSolverModel$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointSetSolverModel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonUserJointSetSolverModel$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointSetSolverModel",
        constants$83.NewtonUserJointSetSolverModel$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointMassScale$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonUserJointMassScale$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointMassScale",
        constants$83.NewtonUserJointMassScale$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointSetFeedbackCollectorCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserJointSetFeedbackCollectorCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointSetFeedbackCollectorCallback",
        constants$83.NewtonUserJointSetFeedbackCollectorCallback$FUNC
    );
}


