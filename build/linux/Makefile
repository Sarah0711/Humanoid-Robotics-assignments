PROJECTS=$(patsubst ../../src/%/CMakeLists.txt,%,$(wildcard ../../src/*/CMakeLists.txt))
CMAKEFILES=$(patsubst %,%/CMakeCache.txt,$(PROJECTS))
PROJECTFILES=$(patsubst %,%/.project,$(PROJECTS))
SETTINGS=$(patsubst %,%/.settings/language.settings.xml,$(PROJECTS))

all: $(CMAKEFILES)

eclipse: $(PROJECTFILES) $(SETTINGS)

define SETTINGSXML
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<project>
        <configuration id="org.eclipse.cdt.core.default.config.1" name="Configuration">
                <extension point="org.eclipse.cdt.core.LanguageSettingsProvider">
                        <provider copy-of="extension" id="org.eclipse.cdt.ui.UserLanguageSettingsProvider"/>
                        <provider-reference id="org.eclipse.cdt.core.ReferencedProjectsLanguageSettingsProvider" ref="shared-provider"/>
                        <provider-reference id="org.eclipse.cdt.core.PathEntryScannerInfoLanguageSettingsProvider" ref="shared-provider"/>
                        <provider-reference id="org.eclipse.cdt.managedbuilder.core.GCCBuiltinSpecsDetector" ref="shared-provider"/>
                </extension>
        </configuration>
</project>
endef
export SETTINGSXML

%/CMakeCache.txt:
	mkdir -p $(dir $@)
	cd $(dir $@) && cmake -DCMAKE_BUILD_TYPE=Debug ../../../src/$(dir $@)

%/.project:
	mkdir -p $(dir $@)
	cd $(dir $@) && cmake ../../../src/$(dir $@) -G"Eclipse CDT4 - Unix Makefiles"
	sed -i 's#<name>.*@\(.*\)</name>#<name>\1</name>#' $@

%/.settings/language.settings.xml:
	mkdir -p $(dir $@)
	echo "$$SETTINGSXML" >> "$@"

.PHONY: all eclipse clean

clean:
	rm -rf $(PROJECTS)
