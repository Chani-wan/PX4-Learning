px4_add_module(
	MODULE examples__task3
    	MAIN task3
    	STACK_MAIN 2000
    	SRCS
        	task3.cpp
    	DEPENDS
)
