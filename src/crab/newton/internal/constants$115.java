// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public final class constants$115 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$115() {}
    public static final FunctionDescriptor NewtonMeshApplyAngleBasedMapping$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshApplyAngleBasedMapping$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshApplyAngleBasedMapping",
        constants$115.NewtonMeshApplyAngleBasedMapping$FUNC
    );
    public static final FunctionDescriptor NewtonCreateTetrahedraLinearBlendSkinWeightsChannel$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateTetrahedraLinearBlendSkinWeightsChannel$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateTetrahedraLinearBlendSkinWeightsChannel",
        constants$115.NewtonCreateTetrahedraLinearBlendSkinWeightsChannel$FUNC
    );
    public static final FunctionDescriptor NewtonMeshOptimize$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshOptimize$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshOptimize",
        constants$115.NewtonMeshOptimize$FUNC
    );
    public static final FunctionDescriptor NewtonMeshOptimizePoints$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshOptimizePoints$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshOptimizePoints",
        constants$115.NewtonMeshOptimizePoints$FUNC
    );
    public static final FunctionDescriptor NewtonMeshOptimizeVertex$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshOptimizeVertex$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshOptimizeVertex",
        constants$115.NewtonMeshOptimizeVertex$FUNC
    );
    public static final FunctionDescriptor NewtonMeshIsOpenMesh$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshIsOpenMesh$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshIsOpenMesh",
        constants$115.NewtonMeshIsOpenMesh$FUNC
    );
}


