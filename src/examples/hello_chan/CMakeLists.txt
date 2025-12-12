px4_add_module(
	MODULE examples__hello_chan
    	MAIN chan
    	STACK_MAIN 2000
    	SRCS
        	chan_main.cpp
    	DEPENDS
)
