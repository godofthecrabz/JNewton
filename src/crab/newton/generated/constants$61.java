// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$61 {

    static final FunctionDescriptor NewtonBodySetMatrixNoSleep$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodySetMatrixNoSleep$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetMatrixNoSleep",
        constants$61.NewtonBodySetMatrixNoSleep$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetMatrixRecursive$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodySetMatrixRecursive$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetMatrixRecursive",
        constants$61.NewtonBodySetMatrixRecursive$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetMaterialGroupID$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonBodySetMaterialGroupID$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetMaterialGroupID",
        constants$61.NewtonBodySetMaterialGroupID$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetContinuousCollisionMode$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonBodySetContinuousCollisionMode$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetContinuousCollisionMode",
        constants$61.NewtonBodySetContinuousCollisionMode$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetJointRecursiveCollision$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonBodySetJointRecursiveCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetJointRecursiveCollision",
        constants$61.NewtonBodySetJointRecursiveCollision$FUNC, false
    );
    static final FunctionDescriptor NewtonBodySetOmega$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBodySetOmega$MH = RuntimeHelper.downcallHandle(
        "NewtonBodySetOmega",
        constants$61.NewtonBodySetOmega$FUNC, false
    );
}

