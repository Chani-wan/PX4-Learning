px4_add_module(
	MODULE examples__trif_posctrl
    	MAIN trif_posctrl
    	STACK_MAIN 2000
	MODULE_CONFIG
		module.yaml
    	SRCS
	    	trif_posctrl.cpp
    	DEPENDS
)
