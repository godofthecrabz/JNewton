// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$88 {

    static final FunctionDescriptor NewtonMeshSaveOFF$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshSaveOFF$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshSaveOFF",
        constants$88.NewtonMeshSaveOFF$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshLoadOFF$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshLoadOFF$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshLoadOFF",
        constants$88.NewtonMeshLoadOFF$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshLoadTetrahedraMesh$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshLoadTetrahedraMesh$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshLoadTetrahedraMesh",
        constants$88.NewtonMeshLoadTetrahedraMesh$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshFlipWinding$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshFlipWinding$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshFlipWinding",
        constants$88.NewtonMeshFlipWinding$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshApplyTransform$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshApplyTransform$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshApplyTransform",
        constants$88.NewtonMeshApplyTransform$FUNC, false
    );
    static final FunctionDescriptor NewtonMeshCalculateOOBB$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonMeshCalculateOOBB$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCalculateOOBB",
        constants$88.NewtonMeshCalculateOOBB$FUNC, false
    );
}


