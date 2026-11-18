// Copyright 2026 rmw_someip contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// QoS events. SOME/IP has no DDS-style status mechanism, so each is derived:
//   matched / incompatible QoS: comparing QoS on the graph channel
//   liveliness changed: the graph for AUTOMATIC, arrival gaps for MANUAL_BY_TOPIC
//   deadline / liveliness lost: the context's QoS watchdog, watching elapsed time

#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"

#include "detail/event_data.hpp"
#include "rmw_someip_cpp/identifier.hpp"

extern "C"
{
rmw_ret_t
rmw_event_set_callback(rmw_event_t * event, rmw_event_callback_t callback, const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(event, RMW_RET_INVALID_ARGUMENT);
  auto * event_data = static_cast<rmw_someip_cpp::EventData *>(event->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_data, RMW_RET_INVALID_ARGUMENT);
  event_data->set_callback(callback, user_data);
  return RMW_RET_OK;
}

bool
rmw_event_type_is_supported(rmw_event_type_t rmw_event_type)
{
  // True only for the kinds whose status is actually maintained. Message lost
  // is not: detecting it needs a sender-side sequence number, and changing the
  // wire format would break CDR compatibility.
  return rmw_someip_cpp::event_type_is_supported(rmw_event_type);
}
}  // extern "C"
