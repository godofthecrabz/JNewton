// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$75 {

    static final FunctionDescriptor NewtonJointGetStiffness$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointGetStiffness$MH = RuntimeHelper.downcallHandle(
        "NewtonJointGetStiffness",
        constants$75.NewtonJointGetStiffness$FUNC, false
    );
    static final FunctionDescriptor NewtonJointSetStiffness$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonJointSetStiffness$MH = RuntimeHelper.downcallHandle(
        "NewtonJointSetStiffness",
        constants$75.NewtonJointSetStiffness$FUNC, false
    );
    static final FunctionDescriptor NewtonDestroyJoint$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonDestroyJoint$MH = RuntimeHelper.downcallHandle(
        "NewtonDestroyJoint",
        constants$75.NewtonDestroyJoint$FUNC, false
    );
    static final FunctionDescriptor NewtonJointSetDestructor$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointSetDestructor$MH = RuntimeHelper.downcallHandle(
        "NewtonJointSetDestructor",
        constants$75.NewtonJointSetDestructor$FUNC, false
    );
    static final FunctionDescriptor NewtonJointIsActive$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointIsActive$MH = RuntimeHelper.downcallHandle(
        "NewtonJointIsActive",
        constants$75.NewtonJointIsActive$FUNC, false
    );
    static final FunctionDescriptor NewtonCreateMassSpringDamperSystem$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCreateMassSpringDamperSystem$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateMassSpringDamperSystem",
        constants$75.NewtonCreateMassSpringDamperSystem$FUNC, false
    );
}


