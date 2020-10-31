#include "EventSystem.h"

int32_t EventSystem::FunscriptActionsChangedEvent = 0;
int32_t EventSystem::FunscriptActionClickedEvent = 0;
int32_t EventSystem::FunscriptSelectionChangedEvent = 0;

int32_t EventSystem::WakeupOnMpvEvents = 0; 
int32_t EventSystem::WakeupOnMpvRenderUpdate = 0;

int32_t EventSystem::FfmpegAudioProcessingFinished = 0;

int32_t EventSystem::MpvVideoLoaded = 0;

int32_t EventSystem::ControllerButtonRepeat = 0;

int32_t EventSystem::SingleShotEvent = 0;

void EventSystem::SingleShotHandler(SDL_Event& ev) noexcept
{
	SingleShotEventData* data = (SingleShotEventData*)ev.user.data1;
	data->handler(data->ctx);
	delete data;
}

void EventSystem::setup()
{
	FunscriptActionsChangedEvent = SDL_RegisterEvents(1);
	FunscriptActionClickedEvent = SDL_RegisterEvents(1);
	FunscriptSelectionChangedEvent = SDL_RegisterEvents(1);
	WakeupOnMpvEvents = SDL_RegisterEvents(1);
	WakeupOnMpvRenderUpdate = SDL_RegisterEvents(1);
	FfmpegAudioProcessingFinished = SDL_RegisterEvents(1);
	MpvVideoLoaded = SDL_RegisterEvents(1);
	ControllerButtonRepeat = SDL_RegisterEvents(1);
	SingleShotEvent = SDL_RegisterEvents(1);

	Subscribe(SingleShotEvent, EVENT_SYSTEM_BIND(this, &EventSystem::SingleShotHandler));
}

void EventSystem::PushEvent(SDL_Event& event)
{
	for (auto& handler : handlers) {
		if (handler.eventType == event.type)
			handler.func(event);
	}
}

void EventSystem::Subscribe(int32_t eventType, void* listener, EventHandlerFunc&& handler)
{
	// this excects the listener to never relocate
	handlers.emplace_back(eventType, listener, handler);
}

void EventSystem::Unsubscribe(int32_t eventType, void* listener)
{
	// this excects the listener to never relocate
	auto it = std::find_if(handlers.begin(), handlers.end(),
		[&](auto& handler) {
			return handler.listener == listener && handler.eventType == eventType;
	});

	if (it != handlers.end()) {
		handlers.erase(it);
	}
	else {
		LOGF_ERROR("Failed to unsubscribe event. \"%d\"", eventType);
		FUN_ASSERT(false, "please investigate");
	}
}