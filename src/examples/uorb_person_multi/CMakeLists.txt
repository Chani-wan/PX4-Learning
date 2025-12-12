px4_add_module(
	MODULE examples__uorb_person_multi
    	MAIN uorb_person_multi
    	STACK_MAIN 2000
    	SRCS
        	uorb_person_multi.cpp
    	DEPENDS

)
