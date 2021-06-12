NAME = gbmu 

# Compiler and linker
CXX = clang++
LD = clang++

# Paths
SRCDIR = src
INCDIR = include
LIBDIR = ..

OBJDIR = obj
BINDIR = .

# Library dependencies
LIBS = $(addprefix $(LIBDIR)/, )

LIBDIRS = $(dir $(LIBS))
LIBINCS = $(addsuffix $(INCDIR), $(LIBDIRS))
LIBARS = $(notdir $(LIBS))

# Sources
INCS = $(LIBINCS) $(INCDIR)
SRCS = $(addprefix $(SRCDIR)/,\
	main.cpp\
)

OBJS = $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
DEPS = $(OBJS:.o=.d)

# Flags
CXXFLAGS = -Wall -Wextra -Werror -Wpedantic -std=c++20 $(INCS:%=-I%)
DFLAGS = -MT $@ -MMD -MP -MF $(OBJDIR)/$*.d
LDFLAGS = $(LIBDIRS:%=-L%)
LDLIBS = $(LIBARS:lib%.a=-l%)

# Compiling commands
COMPILE.cpp = $(CXX) $(DFLAGS) $(CXXFLAGS) -c
COMPILE.o = $(LD) $(LDFLAGS)

all: $(BINDIR)/$(NAME)

# Directories
$(OBJDIR) $(BINDIR):
	@echo "MK $@"
	mkdir -p "$@"

# Libraries
$(LIBS): %.a: FORCE
	make -C $(dir $@) NAME=$(@F)

# Objects
$(OBJS): $(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(OBJDIR)/%.d | $(OBJDIR)
	@mkdir -p '$(@D)'
	@echo "CXX $<"
	$(COMPILE.cpp) $< -o $@

# Dependencies
$(DEPS): $(OBJDIR)/%.d:
include $(wildcard $(DEPS))

# Binaries
$(BINDIR)/$(NAME): $(OBJS) $(LIBS) | $(BINDIR)
	@echo "LD $@ $(LIBARS:lib%.a=-l%)"
	$(COMPILE.o) $< -o $@ $(LDLIBS)

# Remove temporary objects
clean:
	$(foreach libdir, $(LIBDIRS),\
		echo "MK -C $(libdir) $@" && make -C $(libdir) $@ && ):
	@echo "RM $(OBJDIR)"
	rm -rf "$(OBJDIR)"

# Remove all binaries
fclean: clean
	$(foreach libdir, $(LIBDIRS),\
		echo "MK -C $(libdir) $@" && make -C $(libdir) $@ && ):
	@echo "RM $(BINDIR)/$(NAME)"
	rm -f "$(BINDIR)/$(NAME)"
	@rmdir "$(BINDIR)" 2>/dev/null && echo "RM $(BINDIR)" || :

# Remove and rebuild all binaries
re: fclean all

FORCE: ;

.PHONY: all clean fclean re

# Assign a value to VERBOSE to enable verbose output
$(VERBOSE).SILENT:
