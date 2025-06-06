/*
 * ublox_event_observer.h
 * Copyright (C) 2025 Havránek Kryštof <krystof@havrak.xyz>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef UBLOX_EVENT_OBSERVER_H
#define UBLOX_EVENT_OBSERVER_H

#include "ublox_gps.h"
#include "ublox_definitions.h"


class UbloxEventObserver {
	public:
		virtual void onUbloxEvent(const uint8_t msgClass, const uint8_t msgID, const ublox::UBX_message_t* message, const uint16_t msgLen) = 0;

		UbloxEventObserver () = default;
		virtual ~UbloxEventObserver () = default;
		UbloxEventObserver(const UbloxEventObserver&) = delete;
		UbloxEventObserver& operator=(const UbloxEventObserver&) = delete;
};

#endif /* !UBLOX_EVENT_OBSERVER_H */
