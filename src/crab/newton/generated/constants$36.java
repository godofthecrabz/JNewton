// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$36 {

    static final FunctionDescriptor NewtonMaterialGetContactPositionAndNormal$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMaterialGetContactPositionAndNormal$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactPositionAndNormal",
        constants$36.NewtonMaterialGetContactPositionAndNormal$FUNC, false
    );
    static final FunctionDescriptor NewtonMaterialGetContactTangentDirections$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMaterialGetContactTangentDirections$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactTangentDirections",
        constants$36.NewtonMaterialGetContactTangentDirections$FUNC, false
    );
    static final FunctionDescriptor NewtonMaterialGetContactTangentSpeed$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonMaterialGetContactTangentSpeed$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactTangentSpeed",
        constants$36.NewtonMaterialGetContactTangentSpeed$FUNC, false
    );
    static final FunctionDescriptor NewtonMaterialGetContactMaxNormalImpact$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMaterialGetContactMaxNormalImpact$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactMaxNormalImpact",
        constants$36.NewtonMaterialGetContactMaxNormalImpact$FUNC, false
    );
    static final FunctionDescriptor NewtonMaterialGetContactMaxTangentImpact$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonMaterialGetContactMaxTangentImpact$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactMaxTangentImpact",
        constants$36.NewtonMaterialGetContactMaxTangentImpact$FUNC, false
    );
    static final FunctionDescriptor NewtonMaterialGetContactPenetration$FUNC = FunctionDescriptor.of(Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMaterialGetContactPenetration$MH = RuntimeHelper.downcallHandle(
        "NewtonMaterialGetContactPenetration",
        constants$36.NewtonMaterialGetContactPenetration$FUNC, false
    );
}


