// Generated by jextract
package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.lang.foreign.*;
import static java.lang.foreign.ValueLayout.*;
public class constants$57 {

    public static final FunctionDescriptor NewtonCollisionAggregateDestroy$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCollisionAggregateDestroy$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionAggregateDestroy",
        constants$57.NewtonCollisionAggregateDestroy$FUNC
    );
    public static final FunctionDescriptor NewtonCollisionAggregateAddBody$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCollisionAggregateAddBody$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionAggregateAddBody",
        constants$57.NewtonCollisionAggregateAddBody$FUNC
    );
    public static final FunctionDescriptor NewtonCollisionAggregateRemoveBody$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCollisionAggregateRemoveBody$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionAggregateRemoveBody",
        constants$57.NewtonCollisionAggregateRemoveBody$FUNC
    );
    public static final FunctionDescriptor NewtonCollisionAggregateGetSelfCollision$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonCollisionAggregateGetSelfCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionAggregateGetSelfCollision",
        constants$57.NewtonCollisionAggregateGetSelfCollision$FUNC
    );
    public static final FunctionDescriptor NewtonCollisionAggregateSetSelfCollision$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonCollisionAggregateSetSelfCollision$MH = RuntimeHelper.downcallHandle(
        "NewtonCollisionAggregateSetSelfCollision",
        constants$57.NewtonCollisionAggregateSetSelfCollision$FUNC
    );
    public static final FunctionDescriptor NewtonSetEulerAngle$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonSetEulerAngle$MH = RuntimeHelper.downcallHandle(
        "NewtonSetEulerAngle",
        constants$57.NewtonSetEulerAngle$FUNC
    );
}


