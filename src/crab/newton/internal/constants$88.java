// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$88 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$88() {}
    public static final FunctionDescriptor NewtonBodyGetWorld$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetWorld$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetWorld",
        constants$88.NewtonBodyGetWorld$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetCollision$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetCollision",
        constants$88.NewtonBodyGetCollision$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetMaterialGroupID$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetMaterialGroupID$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetMaterialGroupID",
        constants$88.NewtonBodyGetMaterialGroupID$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetSerializedID$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetSerializedID$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetSerializedID",
        constants$88.NewtonBodyGetSerializedID$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetContinuousCollisionMode$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetContinuousCollisionMode$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetContinuousCollisionMode",
        constants$88.NewtonBodyGetContinuousCollisionMode$FUNC
    );
    public static final FunctionDescriptor NewtonBodyGetJointRecursiveCollision$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonBodyGetJointRecursiveCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonBodyGetJointRecursiveCollision",
        constants$88.NewtonBodyGetJointRecursiveCollision$FUNC
    );
}


