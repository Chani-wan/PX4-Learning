px4_add_module(
	MODULE examples__person_example
    	MAIN uorb_person
    	STACK_MAIN 2000
    	SRCS
        	uorb_person.cpp
    	DEPENDS
		px4_work_queue
)
