// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$44 {

	public static final FunctionDescriptor NewtonCompoundCollisionGetNodeIndex$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonCompoundCollisionGetNodeIndex$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionGetNodeIndex",
        constants$44.NewtonCompoundCollisionGetNodeIndex$FUNC, false
    );
	public static final FunctionDescriptor NewtonCompoundCollisionGetCollisionFromNode$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonCompoundCollisionGetCollisionFromNode$MH = RuntimeHelper.downcallHandle(
        "NewtonCompoundCollisionGetCollisionFromNode",
        constants$44.NewtonCompoundCollisionGetCollisionFromNode$FUNC, false
    );
	public static final FunctionDescriptor NewtonCreateFracturedCompoundCollision$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonCreateFracturedCompoundCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateFracturedCompoundCollision",
        constants$44.NewtonCreateFracturedCompoundCollision$FUNC, false
    );
	public static final FunctionDescriptor NewtonFracturedCompoundPlaneClip$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundPlaneClip$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundPlaneClip",
        constants$44.NewtonFracturedCompoundPlaneClip$FUNC, false
    );
	public static final FunctionDescriptor NewtonFracturedCompoundSetCallbacks$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundSetCallbacks$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundSetCallbacks",
        constants$44.NewtonFracturedCompoundSetCallbacks$FUNC, false
    );
	public static final FunctionDescriptor NewtonFracturedCompoundIsNodeFreeToDetach$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFracturedCompoundIsNodeFreeToDetach$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundIsNodeFreeToDetach",
        constants$44.NewtonFracturedCompoundIsNodeFreeToDetach$FUNC, false
    );
}

