// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$74 {

    static final FunctionDescriptor NewtonJointSetUserData$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointSetUserData$MH = RuntimeHelper.downcallHandle(
        "NewtonJointSetUserData",
        constants$74.NewtonJointSetUserData$FUNC, false
    );
    static final FunctionDescriptor NewtonJointGetBody0$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointGetBody0$MH = RuntimeHelper.downcallHandle(
        "NewtonJointGetBody0",
        constants$74.NewtonJointGetBody0$FUNC, false
    );
    static final FunctionDescriptor NewtonJointGetBody1$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointGetBody1$MH = RuntimeHelper.downcallHandle(
        "NewtonJointGetBody1",
        constants$74.NewtonJointGetBody1$FUNC, false
    );
    static final FunctionDescriptor NewtonJointGetInfo$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointGetInfo$MH = RuntimeHelper.downcallHandle(
        "NewtonJointGetInfo",
        constants$74.NewtonJointGetInfo$FUNC, false
    );
    static final FunctionDescriptor NewtonJointGetCollisionState$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonJointGetCollisionState$MH = RuntimeHelper.downcallHandle(
        "NewtonJointGetCollisionState",
        constants$74.NewtonJointGetCollisionState$FUNC, false
    );
    static final FunctionDescriptor NewtonJointSetCollisionState$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonJointSetCollisionState$MH = RuntimeHelper.downcallHandle(
        "NewtonJointSetCollisionState",
        constants$74.NewtonJointSetCollisionState$FUNC, false
    );
}


