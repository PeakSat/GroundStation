#pragma once

#include "Message.hpp"
#include "Peripheral_Definitions.hpp"

struct IdInfo {
    uint8_t messageType; // Just for ADCS
    uint8_t TCTLMID;     // Just for ADCS
    CAN::NodeIDs sourceAddress;
    CAN::NodeIDs destinationAddress;
    bool isMulticast : 1; // Probably not used
};
namespace CAN {
    class TPMessage : public Message {
    public:
        /**
         * The ID information of a CAN-TP Message, as specified in DDJF_OBDH.
         */


        IdInfo idInfo = {};

        TPMessage() = default;

        TPMessage(IdInfo _idInfo) : idInfo(_idInfo){};

        TPMessage(IdInfo _idInfo, bool _isResponse) : idInfo(_idInfo){};

        /**
         * Decodes the ID of a CAN-TP Message, and sets the idInfo field of the current message.
         * @param canID The received ID.
         * @return A struct containing the ID information.
         */
        static IdInfo decodeId(uint32_t canID) {
            IdInfo id_info;
            uint32_t ID = canID;
            id_info.destinationAddress = static_cast<NodeIDs>(ID & 0xFF);
            ID = ID >> 8;
            id_info.sourceAddress = static_cast<NodeIDs>(ID & 0xFF);
            ID = ID >> 8;
            id_info.TCTLMID = ID & 0xFF;
            ID = ID >> 8;
            id_info.messageType = ID & 0x1F;
            return id_info;
        }

        /**
         * Encodes the ID of a CAN-TP Message, using the already set idInfo member.
         * @return The encoded ID.
         */
        static inline uint32_t encodeId(IdInfo id_info) {
            uint32_t id = id_info.messageType << 5;
            id |= id_info.TCTLMID << 8;
            id |= id_info.sourceAddress << 8;
            id |= id_info.destinationAddress;
            return id;
        }
    };
} // namespace CAN
