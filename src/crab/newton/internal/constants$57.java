// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$57 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$57() {}
    public static final FunctionDescriptor NewtonCompoundCollisionGetNodeIndex$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCompoundCollisionGetNodeIndex$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionGetNodeIndex",
        constants$57.NewtonCompoundCollisionGetNodeIndex$FUNC
    );
    public static final FunctionDescriptor NewtonCompoundCollisionGetCollisionFromNode$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCompoundCollisionGetCollisionFromNode$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionGetCollisionFromNode",
        constants$57.NewtonCompoundCollisionGetCollisionFromNode$FUNC
    );
    public static final FunctionDescriptor NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback.class, "apply", constants$57.NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback_UP$FUNC);
    public static final FunctionDescriptor NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$57.NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonCreateFracturedCompoundCollision$emitFracturedCompound$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonCreateFracturedCompoundCollision$emitFracturedCompound_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateFracturedCompoundCollision$emitFracturedCompound_UP$MH = RuntimeHelper.upcallHandle(NewtonCreateFracturedCompoundCollision$emitFracturedCompound.class, "apply", constants$57.NewtonCreateFracturedCompoundCollision$emitFracturedCompound_UP$FUNC);
    public static final FunctionDescriptor NewtonCreateFracturedCompoundCollision$emitFracturedCompound_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCreateFracturedCompoundCollision$emitFracturedCompound_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$57.NewtonCreateFracturedCompoundCollision$emitFracturedCompound_DOWN$FUNC
    );
}


