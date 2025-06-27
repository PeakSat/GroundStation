//
// Created by andro on 5/16/25.
//

#ifndef CANERRORS_HPP
#define CANERRORS_HPP
namespace CAN {
    enum class CANError {
        None,
        Timeout,
        NotInitialized,
        NotReady,
        NotStarted,
        NotSupported,
        Param,
        Pending,
        RamAccess,
        FifoEmpty,
        FifoFull,
        LogOverflow,
        RamWatchdog,
        ProtocolArbitration,
        ProtocolData,
        ReservedArea,
        TTGlobalTime,
        TTTxUnderflow,
        TTTxOverflow,
        TTSchedule1,
        TTSchedule2,
        TTNoInitRef,
        TTNoRef,
        TTAppWatchdog,
        TTConfig,
        Unknown
    };
}
#endif //CANERRORS_HPP
