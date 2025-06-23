ifeq ("$(origin V)", "command line")
  VERBOSE = $(V)
endif
ifeq ($(VERBOSE),1)
  ninja_args = -v -j1
endif

ninja = ninja $(ninja_args)

all: uvc-gadget

BUILDDIR=build/

.PHONY: help
## MakeHelp: Discovered in gphotos-uploader-cli
help: ## Show this help
	@grep -E '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
	 sort | \
	 awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

reconfigure: ## Reconfigure the build
reconfigure: RECONFIGURE=--reconfigure

asan: ## Reconfigure with the address sanitizer enabled
asan: ASAN="-Db_sanitize=address,undefined"
asan: reconfigure

configure: ## Configure the build
$(BUILDDIR)/build.ninja reconfigure configure:
	meson $(BUILDDIR) \
		$(RECONFIGURE) $(ASAN) \
		-Dprefix=/usr

uvc-gadget: ## Build the uvc-gadget library and application
uvc-gadget: | $(BUILDDIR)/build.ninja
	$(ninja) -C $(BUILDDIR)

.PHONY: test install
test: ## Run tests
install: ## Install the package on this system. (Or set DESTDIR)
test install: | $(BUILDDIR)/build.ninja
	$(ninja) -C $(BUILDDIR) $@

iwyu: ## Generate include-what-you-use report
	iwyu_tool -p $(BUILDDIR) -j4 > iwyu.report
	echo "Generated iwyu.report"

.PHONY: package
package: ## Install to a temporary package/ directory
	mkdir -p package
	DESTDIR=$(shell pwd)/package \
		$(ninja) -C $(BUILDDIR) install

clean: ## Remove build entirely
	rm -rf $(BUILDDIR)
