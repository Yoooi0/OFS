#include "OpenFunscripterState.h"
#include "ScriptModeState.h"
#include "Simulator3dState.h"
#include "MetadataEditorState.h"
#include "PreferenceState.h"
#include "SimulatorState.h"
#include "ProjectState.h"
#include "SpecialFunctionsState.h"

void OpenFunscripterState::RegisterAll() noexcept
{
	// App state
	OFS_REGISTER_STATE(OpenFunscripterState);
	OFS_REGISTER_STATE(ScriptingModeState);
	OFS_REGISTER_STATE(Simulator3dState);
	OFS_REGISTER_STATE(FunscriptMetadataState);
	OFS_REGISTER_STATE(PreferenceState);
	OFS_REGISTER_STATE(SimulatorDefaultConfigState);
	OFS_REGISTER_STATE(SpecialFunctionState);

	// Project state
	OFS_REGISTER_STATE(TempoOverlayState);
	OFS_REGISTER_STATE(ProjectState);
	OFS_REGISTER_STATE(ProjectBookmarkState);
	OFS_REGISTER_STATE(SimulatorState);
}

void OpenFunscripterState::addRecentFile(const RecentFile& recentFile) noexcept
{
	FUN_ASSERT(!recentFile.name.empty(), "bruuh");
    auto it = std::find_if(recentFiles.begin(), recentFiles.end(),
		[&](auto& file) {
			return file.projectPath == recentFile.projectPath;
		});
	if (it != recentFiles.end()) {
		*it = recentFile;
	}
	else {
		recentFiles.push_back(recentFile);
	}
	if (recentFiles.size() > 5) {
		recentFiles.erase(recentFiles.begin());
	}
}