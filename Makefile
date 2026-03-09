
GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/cone-calibrator

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	GO_BUILD_FLAGS := -tags no_cgo
	MODULE_BINARY = bin/cone-calibrator.exe
endif

# Cross-compilation: disable CGO when target OS differs from host
HOST_OS := $(shell go env GOOS)
ifneq ($(VIAM_BUILD_OS),)
ifneq ($(VIAM_BUILD_OS), $(HOST_OS))
	GO_BUILD_ENV += CGO_ENABLED=0
	GO_BUILD_FLAGS += -tags no_cgo
endif
endif

$(MODULE_BINARY): Makefile go.mod *.go cmd/module/*.go
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) cmd/module/main.go

lint:
	gofmt -s -w .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY)
ifeq ($(VIAM_BUILD_OS), $(HOST_OS))
ifneq ($(VIAM_TARGET_OS), windows)
	strip $(MODULE_BINARY)
endif
endif
	tar czf $@ meta.json $(MODULE_BINARY)

module: test module.tar.gz

all: test module.tar.gz

setup:
	go mod tidy
