###########################################################################
## Makefile generated for MATLAB file/project 'costotogo'. 
## 
## Makefile     : costotogo_rtw.mk
## Generated on : Wed Jun 11 17:43:17 2014
## MATLAB Coder version: 2.5 (R2013b)
## 
## Build Info:
## 
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/costotogo
## Product type : executable
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPUTER                Computer type. See the MATLAB "computer" command.

PRODUCT_NAME              = costotogo
MAKEFILE                  = costotogo_rtw.mk
COMPUTER                  = GLNXA64
MATLAB_ROOT               = /usr/local/MATLAB/R2013b
MATLAB_BIN                = /usr/local/MATLAB/R2013b/bin
MATLAB_ARCH_BIN           = /usr/local/MATLAB/R2013b/bin/glnxa64
MASTER_ANCHOR_DIR         = 
START_DIR                 = /home/luigi/code/palmieri/learningcost2go
ARCH                      = glnxa64
RELATIVE_PATH_TO_ANCHOR   = /home/luigi/code/palmieri/learningcost2go
ANSI_OPTS                 = -ansi -pedantic -Wno-long-long -fwrapv
CPP_ANSI_OPTS             = -std=c++98 -pedantic -Wno-long-long -fwrapv

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU gcc/g++ v4.4.x | gmake (64-bit Linux)
# Supported Version(s):    4.4.x
# ToolchainInfo Version:   R2013b
# Specification Revision:  1.0
# 


#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# ANSI_OPTS
# CPP_ANSI_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS               = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align
WARN_FLAGS_MAX           = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS           = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align
CPP_WARN_FLAGS_MAX       = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow
MODELREF_LIBS_LINK_GROUP = -Wl,--start-group $(MODELREF_LIBS) -Wl,--end-group

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC = gcc

# Linker: GNU Linker
LD = gcc

# C++ Compiler: GNU C++ Compiler
CPP = g++

# C++ Linker: GNU C++ Linker
CPP_LD = g++

# Archiver: GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_BIN)
MEX = $(MEX_PATH)/mex

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder Application: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = $(MAKE_PATH)/gmake


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @rm -f
ECHO                = @echo
MV                  = @mv
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c $(ANSI_OPTS) -fPIC \
                       -O0
CPPFLAGS             = -c $(CPP_ANSI_OPTS) -fPIC \
                       -O0
CPP_LDFLAGS          = -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)"
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -Wl,--no-undefined
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)"
MEX_CFLAGS           = -MATLAB_ARCH=$(ARCH) $(INCLUDES) \
                         \
                       COPTIMFLAGS="$(ANSI_OPTS)  \
                       -O0 \
                        $(DEFINES)" \
                         \
                       -silent
MEX_LDFLAGS          = LDFLAGS=='$$LDFLAGS'
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -Wl,--no-undefined

#--------------------
# File extensions
#--------------------

H_EXT               = .h
OBJ_EXT             = .o
C_EXT               = .c
EXE_EXT             =
SHAREDLIB_EXT       = .so
HPP_EXT             = .hpp
OBJ_EXT             = .o
CPP_EXT             = .cpp
EXE_EXT             =
SHAREDLIB_EXT       = .so
STATICLIB_EXT       = .a
MEX_EXT             = .mexa64
MAKE_EXT            = .mk


###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/costotogo
PRODUCT_TYPE = "executable"
BUILD_TYPE = "Executable"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(START_DIR)/codegen/exe/costotogo -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_STANDARD = -DMODEL=costotogo -DHAVESTDIO -DUSE_RTMODEL -DUNIX

DEFINES = $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/codegen/exe/costotogo/costotogo_initialize.c $(START_DIR)/codegen/exe/costotogo/costotogo_terminate.c $(START_DIR)/codegen/exe/costotogo/costotogo.c $(START_DIR)/codegen/exe/costotogo/rt_nonfinite.c $(START_DIR)/codegen/exe/costotogo/rtGetNaN.c $(START_DIR)/codegen/exe/costotogo/rtGetInf.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = costotogo_initialize.o costotogo_terminate.o costotogo.o rt_nonfinite.o rtGetNaN.o rtGetInf.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = -lm

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build buildobj clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


buildobj : prebuild $(OBJS) $(PREBUILT_OBJS)
	@echo "### Successfully generated all binary outputs."


prebuild : 


download : build


execute : download
	@echo "### Invoking postbuild tool "Execute" ..."
	$(EXECUTE) $(EXECUTE_FLAGS)
	@echo "### Done invoking postbuild tool."


###########################################################################
## FINAL TARGET
###########################################################################

#-------------------------------------------
# Create a standalone executable            
#-------------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating standalone executable "$(PRODUCT)" ..."
	$(LD) $(LDFLAGS) -o $(PRODUCT) $(OBJS) $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/exe/costotogo/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/exe/costotogo/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : $(MAKEFILE) rtw_proj.tmw


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "###  PRODUCT = $(PRODUCT)"
	@echo "###  PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "###  BUILD_TYPE = $(BUILD_TYPE)"
	@echo "###  INCLUDES = $(INCLUDES)"
	@echo "###  DEFINES = $(DEFINES)"
	@echo "###  ALL_SRCS = $(ALL_SRCS)"
	@echo "###  ALL_OBJS = $(ALL_OBJS)"
	@echo "###  LIBS = $(LIBS)"
	@echo "###  MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "###  SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "###  TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "###  CFLAGS = $(CFLAGS)"
	@echo "###  LDFLAGS = $(LDFLAGS)"
	@echo "###  SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "###  CPPFLAGS = $(CPPFLAGS)"
	@echo "###  CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "###  CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "###  ARFLAGS = $(ARFLAGS)"
	@echo "###  MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "###  MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "###  DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "###  EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "###  MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


