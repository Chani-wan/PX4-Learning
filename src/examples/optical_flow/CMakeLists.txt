px4_add_module(
	MODULE examples__optical_flow
    	MAIN optical_flow_sub
    	STACK_MAIN 2000
    	SRCS
        	optical_flow_sub.cpp
    	DEPENDS
)
