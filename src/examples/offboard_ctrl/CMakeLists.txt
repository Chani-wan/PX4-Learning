px4_add_module(
	MODULE examples__offboard_ctrl
    	MAIN offboard_ctrl
    	STACK_MAIN 2000
    	SRCS
        	offboard_ctrl.cpp
    	DEPENDS
)
