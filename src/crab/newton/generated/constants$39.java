// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$39 {

    static final FunctionDescriptor NewtonMaterialGetContactPruningTolerance$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMaterialGetContactPruningTolerance$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactPruningTolerance",
        constants$39.NewtonMaterialGetContactPruningTolerance$FUNC, false
    );
    static final FunctionDescriptor NewtonMaterialSetContactPruningTolerance$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    static final MethodHandle NewtonMaterialSetContactPruningTolerance$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialSetContactPruningTolerance",
        constants$39.NewtonMaterialSetContactPruningTolerance$FUNC, false
    );
    static final FunctionDescriptor NewtonCreateNull$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCreateNull$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateNull",
        constants$39.NewtonCreateNull$FUNC, false
    );
    static final FunctionDescriptor NewtonCreateSphere$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCreateSphere$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateSphere",
        constants$39.NewtonCreateSphere$FUNC, false
    );
    static final FunctionDescriptor NewtonCreateBox$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCreateBox$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateBox",
        constants$39.NewtonCreateBox$FUNC, false
    );
    static final FunctionDescriptor NewtonCreateCone$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonCreateCone$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateCone",
        constants$39.NewtonCreateCone$FUNC, false
    );
}


