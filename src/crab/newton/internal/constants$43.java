// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
public final class constants$43 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$43() {}
    public static final FunctionDescriptor NewtonWorldGetConstraintCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldGetConstraintCount$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldGetConstraintCount",
        constants$43.NewtonWorldGetConstraintCount$FUNC
    );
    public static final FunctionDescriptor NewtonWorldFindJoint$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonWorldFindJoint$MH = RuntimeHelper.downcallHandle(
        "NewtonWorldFindJoint",
        constants$43.NewtonWorldFindJoint$FUNC
    );
    public static final FunctionDescriptor NewtonIslandGetBody$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonIslandGetBody$MH = RuntimeHelper.downcallHandle(
        "NewtonIslandGetBody",
        constants$43.NewtonIslandGetBody$FUNC
    );
    public static final FunctionDescriptor NewtonIslandGetBodyAABB$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonIslandGetBodyAABB$MH = RuntimeHelper.downcallHandle(
        "NewtonIslandGetBodyAABB",
        constants$43.NewtonIslandGetBodyAABB$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialCreateGroupID$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialCreateGroupID$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialCreateGroupID",
        constants$43.NewtonMaterialCreateGroupID$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialGetDefaultGroupID$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialGetDefaultGroupID$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetDefaultGroupID",
        constants$43.NewtonMaterialGetDefaultGroupID$FUNC
    );
}


