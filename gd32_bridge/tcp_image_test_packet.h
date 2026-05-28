#pragma once

#include "protocol.h"

#include <cstdint>
#include <vector>

namespace gd32_bridge {

std::vector<uint8_t> buildTcpImageTestPacket(uint32_t sequence,
                                             uint16_t width = 256,
                                             uint16_t height = 256,
                                             const CdcDetObjectV2 *detection = nullptr);

} // namespace gd32_bridge
