// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$52 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$52() {}
    public static final FunctionDescriptor NewtonMaterialGetContactPruningTolerance$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMaterialGetContactPruningTolerance$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactPruningTolerance",
        constants$52.NewtonMaterialGetContactPruningTolerance$FUNC
    );
    public static final FunctionDescriptor NewtonMaterialSetContactPruningTolerance$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMaterialSetContactPruningTolerance$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactPruningTolerance",
        constants$52.NewtonMaterialSetContactPruningTolerance$FUNC
    );
    public static final FunctionDescriptor NewtonCreateNull$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateNull$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateNull",
        constants$52.NewtonCreateNull$FUNC
    );
    public static final FunctionDescriptor NewtonCreateSphere$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateSphere$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateSphere",
        constants$52.NewtonCreateSphere$FUNC
    );
    public static final FunctionDescriptor NewtonCreateBox$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateBox$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateBox",
        constants$52.NewtonCreateBox$FUNC
    );
    public static final FunctionDescriptor NewtonCreateCone$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateCone$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateCone",
        constants$52.NewtonCreateCone$FUNC
    );
}


