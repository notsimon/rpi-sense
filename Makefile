PI_ADDR ?= pi@192.168.1.59
PI_FOLDER ?= ~/

PROGRAMS = altimeter test_lsp25h
MODULES = src/led_matrix src/lsp25h
LIBS = m

CXX ?= g++
CXXFLAGS += -Wall -std=c++11
LDFLAGS += $(addprefix -l, $(LIBS))

BUILD_DIR = build

ifdef DEBUG
	CXXFLAGS += -O0 -g
	BUILD_DIR := $(BUILD_DIR)/debug
else ifdef PROFILE
	CXXFLAGS += -O3 -DNDEBUG -pg
	LDFLAGS += -pg
	BUILD_DIR := $(BUILD_DIR)/profile
else
	CXXFLAGS += -O3 -DNDEBUG
	BUILD_DIR := $(BUILD_DIR)/release
endif

EXES = $(addprefix $(BUILD_DIR)/, $(PROGRAMS))
OBJS = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(MODULES)))

define link-program
	@echo "  LD" $@
	@$(CXX) $^ $(LDFLAGS) -o $@
endef

all : $(EXES)
	$(info Build directory: $(BUILD_DIR))
	$(info CXXFLAGS: $(CXXFLAGS))
	$(info LDFLAGS: $(LDFLAGS))

# dependencies

$(BUILD_DIR) :
	mkdir -p $@

$(BUILD_DIR)/%.d : %.cc | $(BUILD_DIR)
	@mkdir -p $(@D)
	@echo "  DEPS" $<
	@$(CXX) $(CXXFLAGS) -MM -MP -MG $< -MF $@ -MT '$$(BUILD_DIR)/$*.o $$(BUILD_DIR)/$*.d'
	@sed -i '' 's/ \([^/ ]*\.hp\{0,2\}\)/ $(subst /,\/,$(<D))\/\1/g' $@ # prepend src/ to the missing headers

include $(subst .o,.d, $(OBJS))

# build

$(BUILD_DIR)/%.o : %.cc $(BUILD_DIR)/%.d Makefile | $(BUILD_DIR)
	@echo "  CXX" $<
	@$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/% : $(BUILD_DIR)/src/%.o $(OBJS)
	$(link-program)

push : $(EXES)
ifndef PI_ADDR
	$(error PI_ADDR not defined)
endif
	scp -r $^ $(PI_ADDR):$(PI_FOLDER)

clean :
	rm -rvf build

