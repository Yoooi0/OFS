#pragma once

#include "imgui.h"

#include "OFS_Reflection.h"

class ScriptSimulator {
private:
	ImVec2 startDragP1;
	ImVec2 startDragP2;
	ImVec2* dragging = nullptr;
	bool movingBar = false;
	bool EnableIndicators = true;
	bool EnableVanilla = false;
	bool EnablePosition = false;
	const bool ShowMovementHandle = false;

	inline int32_t GetColor(const ImColor& col) const noexcept {
		auto color = ImGui::ColorConvertFloat4ToU32(col);
		// apply global opacity
		((uint8_t*)&color)[IM_COL32_A_SHIFT / 8] = ((uint8_t)(255 * col.Value.w * simulator.GlobalOpacity));
		return color;
	}
public:
	struct SimulatorSettings {
		ImVec2 P1;
		ImVec2 P2;
		ImColor Text = IM_COL32(0xFF, 0xFF, 0xFF, 0xFF);
		ImColor Front = IM_COL32(0x01, 0xBA, 0xEF, 0xFF);
		ImColor Back = IM_COL32(0x10, 0x10, 0x10, 0xFF);
		ImColor Border = IM_COL32(0x0B, 0x4F, 0x6C, 0xFF);
		ImColor Indicator = IM_COL32(0xFF, 0x4F, 0x6C, 0xFF);
		float Width = 120.f;
		float BorderWidth = 8.f;
		float GlobalOpacity = 1.f;

		template <class Archive>
		inline void reflect(Archive& ar)
		{
			OFS_REFLECT(P1, ar);
			OFS_REFLECT(P2, ar);
			OFS_REFLECT(Width, ar);
			OFS_REFLECT(BorderWidth, ar);
			OFS_REFLECT(Text, ar);
			OFS_REFLECT(Front, ar);
			OFS_REFLECT(Back, ar);
			OFS_REFLECT(Border, ar);
			OFS_REFLECT(Indicator, ar);
			OFS_REFLECT(GlobalOpacity, ar);
		}
	} simulator;
	bool SimulateRawActions = false;

	ScriptSimulator() {}
	void setup();
	void CenterSimulator();
	void ShowSimulator(bool* open);
};