# ilxctl - Sony ILX-LR1 control panel
#
# The Camera Remote SDK ships as prebuilt universal dylibs whose install name is
# @rpath/..., so the build stages everything into build/ and links with an
# @executable_path rpath.
#
# On macOS the SDK resolves its transport adapters through the app-bundle
# convention, i.e. <exe_dir>/Contents/Frameworks/CrAdapter. Staging them only
# in <exe_dir>/CrAdapter makes EnumCameraObjects fail with 0x8703
# (CrError_Adaptor_Create), so both locations are populated - which is exactly
# what Sony's own CMakeLists does for Apple builds.

BUILD    := build
BIN      := $(BUILD)/ilxctl
SRCS     := src/main.cpp src/camera.cpp
OBJS     := $(patsubst src/%.cpp,$(BUILD)/obj/%.o,$(SRCS))
DEPS     := $(OBJS:.o=.d)

CXX      ?= clang++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -MMD -MP \
            -isystem include -isystem third_party -Isrc -mmacosx-version-min=12.1
LDFLAGS  := -Llib -lCr_Core -Wl,-rpath,@executable_path -pthread

SDK_LIBS := $(wildcard lib/*.dylib)
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
	@cp "$(CRSDK_DIR)"/external/crsdk/*.dylib lib/
	@cp -R "$(CRSDK_DIR)/external/crsdk/CrAdapter" lib/
	@xattr -dr com.apple.quarantine include lib 2>/dev/null || true
	@echo "SDK staged from $(CRSDK_DIR)"

sdk-check:
	@test -f include/CRSDK/CameraRemote_SDK.h && test -f lib/libCr_Core.dylib || { \
	  echo "The Sony Camera Remote SDK is missing (it is not redistributed here)."; \
	  echo "Run:  make sdk CRSDK_DIR=/path/to/unpacked/RemoteCli"; \
	  exit 1; }

$(BUILD)/obj/%.o: src/%.cpp | $(BUILD)/obj
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN): $(OBJS) | $(BUILD)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@

$(BUILD) $(BUILD)/obj:
	@mkdir -p $@

# The SDK dylibs must sit beside the binary, un-quarantined, or dyld refuses
# to load them.
$(BUILD)/%.dylib: lib/%.dylib | $(BUILD)
	@cp $< $@
	@xattr -d com.apple.quarantine $@ 2>/dev/null || true

$(BUILD)/.adapters: $(wildcard lib/CrAdapter/*.dylib) | $(BUILD)
	@mkdir -p $(BUILD)/CrAdapter $(BUILD)/Contents/Frameworks/CrAdapter
	@cp lib/CrAdapter/*.dylib $(BUILD)/CrAdapter/
	@cp lib/CrAdapter/*.dylib $(BUILD)/Contents/Frameworks/CrAdapter/
	@xattr -dr com.apple.quarantine $(BUILD)/CrAdapter $(BUILD)/Contents 2>/dev/null || true
	@touch $@

run: all
	@$(BIN)

clean:
	rm -rf $(BUILD)

rebuild: clean all

-include $(DEPS)
