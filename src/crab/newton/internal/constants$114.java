// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$114 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$114() {}
    public static final FunctionDescriptor NewtonMeshCalculateVertexNormals$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT
    );
    public static final MethodHandle NewtonMeshCalculateVertexNormals$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshCalculateVertexNormals",
        constants$114.NewtonMeshCalculateVertexNormals$FUNC
    );
    public static final FunctionDescriptor NewtonMeshApplySphericalMapping$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshApplySphericalMapping$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshApplySphericalMapping",
        constants$114.NewtonMeshApplySphericalMapping$FUNC
    );
    public static final FunctionDescriptor NewtonMeshApplyCylindricalMapping$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshApplyCylindricalMapping$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshApplyCylindricalMapping",
        constants$114.NewtonMeshApplyCylindricalMapping$FUNC
    );
    public static final FunctionDescriptor NewtonMeshApplyBoxMapping$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshApplyBoxMapping$MH = RuntimeHelper.downcallHandle(
        "NewtonMeshApplyBoxMapping",
        constants$114.NewtonMeshApplyBoxMapping$FUNC
    );
    public static final FunctionDescriptor NewtonMeshApplyAngleBasedMapping$reportPrograssCallback$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonMeshApplyAngleBasedMapping$reportPrograssCallback_UP$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshApplyAngleBasedMapping$reportPrograssCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonMeshApplyAngleBasedMapping$reportPrograssCallback.class, "apply", constants$114.NewtonMeshApplyAngleBasedMapping$reportPrograssCallback_UP$FUNC);
    public static final FunctionDescriptor NewtonMeshApplyAngleBasedMapping$reportPrograssCallback_DOWN$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_FLOAT$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonMeshApplyAngleBasedMapping$reportPrograssCallback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$114.NewtonMeshApplyAngleBasedMapping$reportPrograssCallback_DOWN$FUNC
    );
}


