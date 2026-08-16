# ilxctl - Sony ILX-LR1 control panel
#
# Builds on macOS (Apple Silicon) and Linux (aarch64, e.g. Jetson Orin).
#
# The Camera Remote SDK ships as prebuilt shared libraries that resolve their
# siblings through an rpath relative to the loading binary, so the build stages
# the SDK next to the executable on both platforms:
#
#   macOS   dylibs, install name @rpath/..., link with -rpath @executable_path.
#           The SDK resolves its transport adapters through the app-bundle
#           convention, i.e. <exe_dir>/Contents/Frameworks/CrAdapter. Staging
#           them only in <exe_dir>/CrAdapter makes EnumCameraObjects fail with
#           0x8703 (CrError_Adaptor_Create), so both locations are populated -
#           which is exactly what Sony's own CMakeLists does for Apple builds.
#           The quarantine attribute must also be stripped or dyld refuses to
#           load the dylibs.
#
#   Linux   .so files with RUNPATH=$$ORIGIN baked in already; link with
#           -rpath '$$ORIGIN'. The adapters live in <exe_dir>/CrAdapter only -
#           the Contents/Frameworks duplication is an Apple-only convention, and
#           there is no quarantine attribute. Matches the NOT APPLE branch of
#           Sony's CMakeLists.

UNAME_S := $(shell uname -s)

BUILD    := build
BIN      := $(BUILD)/ilxctl
SRCS     := src/main.cpp src/camera.cpp
OBJS     := $(patsubst src/%.cpp,$(BUILD)/obj/%.o,$(SRCS))
DEPS     := $(OBJS:.o=.d)

CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -MMD -MP \
            -isystem include -isystem third_party -Isrc
LDFLAGS  := -Llib -lCr_Core -pthread

ifeq ($(UNAME_S),Darwin)
  CXX      ?= clang++
  DYLIB_EXT := dylib
  CXXFLAGS += -mmacosx-version-min=12.1
  LDFLAGS  += -Wl,-rpath,@executable_path
else
  CXX      ?= g++
  DYLIB_EXT := so
  # $$ORIGIN reaches make as $ORIGIN, which is what the dynamic linker wants.
  LDFLAGS  += -Wl,-rpath,'$$ORIGIN' -Wl,--disable-new-dtags
  # char is unsigned by default on ARM, so the SDK's CrInt8-backed enums (which
  # use -1 for Wide) will not compile without this. Sony's own CMakeLists sets
  # the same two flags for UNIX AND NOT APPLE.
  CXXFLAGS += -fsigned-char -fstack-protector-all
  # Tune for the host CPU (Cortex-A78AE on Orin). Silently skipped on
  # toolchains that reject it, so a generic build still works.
  CXXFLAGS += $(shell $(CXX) -mcpu=native -E -x c++ /dev/null >/dev/null 2>&1 \
                && echo -mcpu=native)
endif

SDK_LIBS := $(wildcard lib/*.$(DYLIB_EXT))
STAGED   := $(patsubst lib/%,$(BUILD)/%,$(SDK_LIBS)) $(BUILD)/.adapters

.PHONY: all run clean rebuild sdk sdk-check

all: sdk-check $(BIN) $(STAGED)

# The Sony SDK is not redistributed with this repo. Point CRSDK_DIR at an
# unpacked RemoteCli/SimpleCli package to populate include/CRSDK and lib/.
CRSDK_DIR ?= ../RemoteCli

sdk:
	@test -d "$(CRSDK_DIR)/app/CRSDK" || { \
	  echo "Not found: $(CRSDK_DIR)/app/CRSDK"; \
	  echo "Unpack the Camera Remote SDK and re-run:  make sdk CRSDK_DIR=/path/to/RemoteCli"; \
	  exit 1; }
	@mkdir -p include lib
	@cp -R "$(CRSDK_DIR)/app/CRSDK" include/
	@cp "$(CRSDK_DIR)"/external/crsdk/*.$(DYLIB_EXT) lib/
	@cp -R "$(CRSDK_DIR)/external/crsdk/CrAdapter" lib/
ifeq ($(UNAME_S),Darwin)
	@xattr -dr com.apple.quarantine include lib 2>/dev/null || true
endif
	@echo "SDK staged from $(CRSDK_DIR)"

sdk-check:
	@test -f include/CRSDK/CameraRemote_SDK.h && test -f lib/libCr_Core.$(DYLIB_EXT) || { \
	  echo "The Sony Camera Remote SDK is missing (it is not redistributed here)."; \
	  echo "Run:  make sdk CRSDK_DIR=/path/to/unpacked/RemoteCli"; \
	  exit 1; }

$(BUILD)/obj/%.o: src/%.cpp | $(BUILD)/obj
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN): $(OBJS) | $(BUILD)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@

$(BUILD) $(BUILD)/obj:
	@mkdir -p $@

# The SDK libraries must sit beside the binary (and on macOS be un-quarantined,
# or dyld refuses to load them).
$(BUILD)/%.$(DYLIB_EXT): lib/%.$(DYLIB_EXT) | $(BUILD)
	@cp $< $@
ifeq ($(UNAME_S),Darwin)
	@xattr -d com.apple.quarantine $@ 2>/dev/null || true
endif

$(BUILD)/.adapters: $(wildcard lib/CrAdapter/*.$(DYLIB_EXT)) | $(BUILD)
	@mkdir -p $(BUILD)/CrAdapter
	@cp lib/CrAdapter/*.$(DYLIB_EXT) $(BUILD)/CrAdapter/
ifeq ($(UNAME_S),Darwin)
	@mkdir -p $(BUILD)/Contents/Frameworks/CrAdapter
	@cp lib/CrAdapter/*.dylib $(BUILD)/Contents/Frameworks/CrAdapter/
	@xattr -dr com.apple.quarantine $(BUILD)/CrAdapter $(BUILD)/Contents 2>/dev/null || true
endif
	@touch $@

run: all
	@$(BIN)

clean:
	rm -rf $(BUILD)

rebuild: clean all

-include $(DEPS)
