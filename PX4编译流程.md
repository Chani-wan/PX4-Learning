<h3 align="center"> PX4二次开发 </h3>

<div align="center">
    <h4 style="background-color: #FFFF00; display: inline-block; padding: 5px;">PX4编译流程</h4>
</div>

#### **1、编译工具**

- CMake：指定编译规则[怎么编译？有哪些依赖？
- KConfig：告诉PX4，是否要将该程序编译进去？
- make：代码编译,执行Makefile里的对应命令。

#### **2、编译流程**

​	我们第一步执行了 `make hkust_nxt-dual` ,然后我们去 `Makefile` 里查找目标 `hkust_nxt-dual` ,最终找到以下几行代码：
```makefile
# Get a list of all config targets boards/*/*.px4board  
ALL_CONFIG_TARGETS := $(shell find boards -maxdepth 3 -mindepth 3 -name '*.px4board' -print | sed -e 's|boards\/||' | sed -e 's|\.px4board||' | sed -e 's|\/|_|g' | sort)  
```

​	先看第一段命令：`find boards -maxdepth 3 -mindepth 3 -name '*.px4board'` ,==它的作用是在`board`路径下查找所有以`.px4board`结尾的文件，搜索深度指定为3==。后面的两条命令是对这些文件进行字符串处理：把`boards/`替换为空字符串，把`.px4board`替换为空字符串，把`/`替换为`_` ，因此根据这个规则我们最终可以找到`hkust_nxt-dual_*`的身影。接下来我们看下面一段代码：
```makefile
# All targets.
$(ALL_CONFIG_TARGETS):
	@$(call cmake-build,$@$(BUILD_DIR_SUFFIX)) 
```

表明上述的所有找到的结果都是一个目标，仔细查找会发现，我们在上面找到的所有目标里面并没有这个`hkust_nxt-dual` ，那这个目标在哪里呢？答案在下一句代码里：
```makefile
# Filter for only default targets to allow omiting the "_default" postfix
CONFIG_TARGETS_DEFAULT := $(patsubst %_default,%,$(filter %_default,$(ALL_CONFIG_TARGETS)))
$(CONFIG_TARGETS_DEFAULT):
	@$(call cmake-build,$@_default$(BUILD_DIR_SUFFIX))
```

这一句代码首先通过`patsubst`和`filter`两条Makefile命令，将上面找到的所有目标中，以`_default`结尾的目标，去掉`_default`后缀，并替换为空串。==也就是说：不带任何后缀的目标名称，默认后缀是`_default`。==然后他会执行`call cmake-build` ，这是一个函数，实现在这里：
```makefile
define cmake-build                               
	$(eval override CMAKE_ARGS += -DCONFIG=$(1))
	@$(eval BUILD_DIR = "$(SRC_DIR)/build/$(1)")
	@# check if the desired cmake configuration matches the cache then CMAKE_CACHE_CHECK stays empty
	@$(call cmake-cache-check)
	@# make sure to start from scratch when switching from GNU Make to Ninja
	@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e $(BUILD_DIR)/Makefile ]; then rm -rf $(BUILD_DIR); fi
	@# make sure to start from scratch if ninja build file is missing
	@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ ! -f $(BUILD_DIR)/build.ninja ]; then rm -rf $(BUILD_DIR); fi
	@# only excplicitly configure the first build, if cache file already exists the makefile will rerun cmake automatically if necessary
	@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ] || [ $(CMAKE_CACHE_CHECK) ]; then \
		mkdir -p $(BUILD_DIR) \
		&& cd $(BUILD_DIR) \
		&& cmake "$(SRC_DIR)" -G"$(PX4_CMAKE_GENERATOR)" $(CMAKE_ARGS) \ 
		|| (rm -rf $(BUILD_DIR)); \
	fi
	@# run the build for the specified target
	@cmake --build $(BUILD_DIR) -- $(PX4_MAKE_ARGS) $(ARGS)
endef
```

​	我们可以看到，这里最终调用的是 `cmake`来进行工程构建，并且会把目标名称传递给`cmake`作为参数，名字叫做`CONFIG`，以我们的编译流程为例，此时传递的`CONFIG`参数就是`hkust_nxt-dual_default`。

#### **3、目标解析和参数配置**

​	目标传递进入cmake之后，cmake会进一步对这个参数进行解析，然后找到对应的配置文件，这个功能写在 `cmake/px4_config.cmake` ,这里首先查找所有的配置文件，然后依次看，每一个配置文件哪一个在变换成目标名称之后和`CONFIG`匹配，从而进一步找到对应配置文件。以我们的编译流程为例，==我们的编译目标是`hkust_nxt-dual_default`，那么最终被找到的配置文件就是：`boards/hkust/nxt-dual/default.px4board`。==

​	还记得我们在进行example编译的时候，第一步做了什么吗？我们先执行了：`
make hkust_nxt-dual boardconfig
`，执行之后启动了一个可视化配置界面，我们进去将`CONFIG_EXAMPLES_HELLO_PX4`设置为true。这一步最终的影响，无非就是会对这个`default.px4board`文件进行处理，把对应的变量设置为true并将新的内容写入这个文件，所以我们会看到，有的教程会教我们直接改这个文件，在文件末尾加上对应的配置然后进行编译，无需执行`boardconfig`这一步。两种方式都行，看你自己喜欢哪一种方式。

​	这个配置文件正是给Kconfig这个工具使用的，我们在Kconfig文件里定一个每一个变量，在这里都可以进行配置。在`cmake/kconfig.cmake`里面，我们看到这样一段代码：

```makefile
if(EXISTS ${BOARD_DEFCONFIG})

	# Depend on BOARD_DEFCONFIG so that we reconfigure on config change
	set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${BOARD_DEFCONFIG})

	if(${LABEL} MATCHES "default" OR ${LABEL} MATCHES "performance-test" OR ${LABEL} MATCHES "bootloader" OR ${LABEL} MATCHES "canbootloader")
		# Generate boardconfig from saved defconfig
		execute_process(
			COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
			${DEFCONFIG_PATH} ${BOARD_DEFCONFIG}
			WORKING_DIRECTORY ${PX4_SOURCE_DIR}
			OUTPUT_VARIABLE DUMMY_RESULTS
		)
	else()
		# Generate boardconfig from default.px4board and {label}.px4board
		execute_process(
			COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/merge_config.py Kconfig ${BOARD_CONFIG} ${PX4_BOARD_DIR}/default.px4board ${BOARD_DEFCONFIG}
			WORKING_DIRECTORY ${PX4_SOURCE_DIR}
			OUTPUT_VARIABLE DUMMY_RESULTS
		)
	endif()

	if(${LABEL} MATCHES "allyes")
		message(AUTHOR_WARNING "allyes build: allyes is for CI coverage and not for use in production")
		execute_process(
			COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/allyesconfig.py
			WORKING_DIRECTORY ${PX4_SOURCE_DIR}
		)
	endif()

```

9这一段代码会先调用Kconfig工具对这个配置文件进行处理，并且在**构建目录**下生成这样几个文件：`boardconfig` ，`px4_boardconfig.h`，这是一堆的C++预编译指令，这两个文件各有用处。

#### **4、boardconfig文件的处理**

​	我们还有一个问题没有找到答案：PX4的编译系统到底是怎么控制我们对应的CONFIG变量设置为true就编译，设置为false就不编译呢？答案其实还在cmake里面，在`cmake/kconfig.cmake`中，我们看到这样一个迭代：

```makefile
# parse board config options for cmake
	file(STRINGS ${BOARD_CONFIG} ConfigContents)
	foreach(NameAndValue ${ConfigContents})
		# Strip leading spaces
		string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

		# Find variable name
		string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

```

​	这里我们看到，这个迭代是对生成的boardconfig文件的每一行文件内容进行处理，并且会通过正则表达式去处理每一个形如`CONFIG_xxxxx=xxxxx`的行（回顾一下boardconfig的文件内容，是不是这个样子？）。我们将迭代往下翻，会看到这样一段：
```makefile
# Find variable name
		string(REGEX MATCH "^CONFIG_BOARD_" Board ${NameAndValue})

		if(Board)
			string(REPLACE "CONFIG_BOARD_" "" ConfigKey ${Name})
			if(Value)
				set(${ConfigKey} ${Value})
				message(STATUS "${ConfigKey} ${Value}")
			endif()
		endif()
```

​	我们看到，它会处理每一个形如`CONFIG_EXAMPLES_xxxx`的配置项，并且将example的名称提取出来之后，转成小写，然后将`examples/${example}`加入到`config_module_list`里面。还是以我们写的example为例，我们看到boardconfig里面有这样一句话：`CONFIG_EXAMPLES_HELLO_PX4=y`刚好满足上面的正则表达式，cmake在处理到该文件的这一行之后，会将example的名称，也就是`HELLO_PX4`提取出来，之后将它转成小写变成`hello_px4`，然后将`examples/hello_px4`加入到`config_module_list`之中。这个变量怎么用呢？在`CMakeLists.txt`里面，有这样一段代码：

```makefile
foreach(module ${config_module_list})
	add_subdirectory(src/${module})
endforeach()
```

​	对于每一个`config_module_list`中的元素，都调用cmake的`add_subdirectory`命令，将对应的子目录加进去进行编译。以我们的example为例，到这里cmake最终是执行了`add_subdirectory(src/examples/hello_px4)`。到这一步，后面的事情就非常明白了，我们创建的example子目录被cmake加进去一起处理，那么我们在这个目录下写的CMakelists.txt就起了作用，最终我们的代码就被编译进去了。反之，这个目录不会被加进去，自然也就不会被编译。

#### **5、px4_boardconfig.h的作用**

​	这个比较简单，就是在代码里用宏作为功能控制开关，我们在代码里会看到一大堆这样用宏控制的代码，这些宏都是在自动生成的头文件里被定义的。