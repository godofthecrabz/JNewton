// Generated by jextract
package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public class constants$86 {

    public static final FunctionDescriptor NewtonUserJoinRowsCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserJoinRowsCount$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJoinRowsCount",
        constants$86.NewtonUserJoinRowsCount$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointGetGeneralRow$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonUserJointGetGeneralRow$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointGetGeneralRow",
        constants$86.NewtonUserJointGetGeneralRow$FUNC
    );
    public static final FunctionDescriptor NewtonUserJointGetRowForce$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonUserJointGetRowForce$MH = RuntimeHelper.downcallHandle(
        "NewtonUserJointGetRowForce",
        constants$86.NewtonUserJointGetRowForce$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreate$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreate$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreate",
        constants$86.NewtonMeshCreate$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreateFromMesh$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreateFromMesh$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateFromMesh",
        constants$86.NewtonMeshCreateFromMesh$FUNC
    );
    public static final FunctionDescriptor NewtonMeshCreateFromCollision$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshCreateFromCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCreateFromCollision",
        constants$86.NewtonMeshCreateFromCollision$FUNC
    );
}


