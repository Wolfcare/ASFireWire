#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>

#include "../ResponseCode.hpp"
#include "../../Bus/Timing/SplitTimeout.hpp"
#include <memory>
#include "../Rx/PacketRouter.hpp"

namespace ASFW::Async {

class DescriptorBuilder;
class ATResponseContext;
class IFireWireBusInfo;
namespace Bus { class GenerationTracker; }
namespace Engine { class ContextManager; }
namespace Tx { class Submitter; }

/// The speed a request arrived at, from its AR trailer: xferStatus is the
/// trailer's upper half, and the speed sits in trailer bits 23:21. A response
/// goes back at that speed, as Linux sends it (ohci.c handle_ar_packet:
/// p.speed = (status >> 21) & 0x7; core-transaction.c: request->response.speed
/// = p->speed). A fixed S400 is never acknowledged by a node whose link runs
/// slower, such as the MOTU UltraLite mk3 (link_spd S100).
[[nodiscard]] constexpr uint8_t RequestSpeed(uint16_t xferStatus) noexcept {
    return static_cast<uint8_t>((xferStatus >> 5) & 0x07);
}

/// Whether a received request waits for a response: our link acknowledged it
/// with ack_pending. The AR trailer's event code is the ack plus 16
/// (ohci.c handle_ar_packet: p.ack = evt - 16); ack_pending is 2, so 0x12.
/// Linux responds only then (core-transaction.c fw_send_response).
[[nodiscard]] constexpr bool ExpectsResponse(uint16_t xferStatus) noexcept {
    return (xferStatus & 0x1f) == 0x12;
}

/// Utility to build and send Write Response (WrResp) packets for incoming AR requests.
class ResponseSender {
public:
    ResponseSender(DescriptorBuilder& builder,
                   Tx::Submitter& submitter,
                   Engine::ContextManager& ctxMgr,
                   Bus::GenerationTracker& generationTracker,
                   std::shared_ptr<FW::SplitTimeout> splitTimeout = std::make_shared<FW::SplitTimeout>()) noexcept;

    /// Build and transmit a WrResp for the given request packet.
    /// Skips transmission for broadcast requests (destID=0xFFFF).
    void SendWriteResponse(const ARPacketView& request, ResponseCode rcode) noexcept;

    /// Build and transmit a Read Quadlet Response (tCode 0x6).
    void SendReadQuadletResponse(const ARPacketView& request,
                                 ResponseCode rcode,
                                 uint32_t quadletData) noexcept;

    /// Build and transmit a Read Block Response (tCode 0x7).
    void SendReadBlockResponse(const ARPacketView& request,
                               ResponseCode rcode,
                               uint64_t payloadDeviceAddress,
                               uint32_t payloadLength) noexcept;

    /// Build and transmit a Lock Response (tCode 0xB) for compare-swap requests.
    void SendLockResponse(const ARPacketView& request,
                          ResponseCode rcode,
                          uint32_t oldValue) noexcept;

private:
    struct ScratchRegion {
        std::byte* base{nullptr};
        uint64_t deviceBase{0};
        uint32_t slotCount{0};
        std::atomic<uint32_t> nextSlot{0};
    };

    void SendResponse(const ARPacketView& request,
                      ResponseCode rcode,
                      uint8_t responseTCode,
                      const uint32_t* header,
                      std::size_t headerBytes,
                      uint64_t payloadDeviceAddress,
                      std::size_t payloadLength) noexcept;

    DescriptorBuilder& builder_;
    Tx::Submitter& submitter_;
    Engine::ContextManager& ctxMgr_;
    Bus::GenerationTracker& generationTracker_;
    std::shared_ptr<FW::SplitTimeout> splitTimeout_;
    ScratchRegion lockResponseScratch_;
};

} // namespace ASFW::Async
