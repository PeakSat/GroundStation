#include "TransceiverTask.hpp"
#include "Logger.hpp"
using namespace AT86RF215;

extern uint8_t button_flag;
extern SPI_HandleTypeDef hspi4;

AT86RF215::At86rf215 TransceiverTask::transceiver = AT86RF215::At86rf215(&hspi4, AT86RF215::AT86RF215Configuration());

uint8_t TransceiverTask::checkTheSPI() {
    uint8_t spi_error = 0;
    DevicePartNumber dpn = transceiver.get_part_number(error);
    if(dpn == DevicePartNumber::AT86RF215)
        LOG_DEBUG << "SPI OK" ;
    else{
        spi_error = 1;
        LOG_DEBUG << "SPI ERROR" ;
        transceiver.chip_reset(error);
    }
    return spi_error;
}

TransceiverTask::PacketType TransceiverTask::createRandomPacket(uint16_t length) {
    PacketType packet;
    packet[0] = packet_id;
    for (std::size_t i = 1; i < length; i++) {
        packet[i] = i;
    }
    return packet;
}

void TransceiverTask::setConfiguration(uint16_t pllFrequency09, uint8_t pllChannelNumber09) {
    CustomConfig.pllFrequency09 = pllFrequency09;
    CustomConfig.pllChannelNumber09 = pllChannelNumber09;
    CustomConfig.pllChannelMode09 = AT86RF215::PLLChannelMode::FineResolution450;
    CustomConfig.transceiverSampleRate09 = AT86RF215::TransmitterSampleRate::FS_400;
    CustomConfig.continuousTransmit09 = false;
    CustomConfig.baseBandEnable09 = true;
    CustomConfig.physicalLayerType09 = PhysicalLayerType::BB_MRFSK;
    CustomConfig. frameCheckSequenceType09 = FrameCheckSequenceType::FCS_32;
    CustomConfig.txOutPower09 = 0;
    transceiver.config = CustomConfig;
}

void TransceiverTask::receiverConfig(bool agc_enable){
    transceiver.setup_rx_frontend(AT86RF215::RF09,
                                  false,
                                  false,
                                  AT86RF215::ReceiverBandwidth::RF_BW200KHZ_IF250KHZ,
                                  AT86RF215::RxRelativeCutoffFrequency::FCUT_0375,
                                  AT86RF215::ReceiverSampleRate::FS_400,
                                  false,
                                  AT86RF215::AverageTimeNumberSamples::AVGS_8,
                                  false,
                                  AT86RF215::AutomaticGainTarget::DB30,
                                  23,
                                  error);
    // ENABLE AGC
    RegisterAddress regagcc = AT86RF215::RF09_AGCC;
    uint8_t reg = transceiver.spi_read_8(regagcc, error);
    reg = reg | (static_cast<uint8_t>(agc_enable));
    transceiver.spi_write_8(regagcc, reg, error);

    transceiver.setup_rssi(AT86RF215::RF09,
                           AT86RF215::EnergyDetectionMode::RF_EDAUTO,
                           16,
                           AT86RF215::EnergyDetectionTimeBasis::RF_8MS,
                           error);
}

/*
* The frequency cannot be lower than 377000 as specified in section 6.3.2. The frequency range related
* to Fine Resolution Channel Scheme CNM.CM=1 is from 389.5MHz to 510MHz
*/
uint16_t TransceiverTask::calculatePllChannelFrequency09(uint32_t frequency) {
    uint32_t N = (frequency - 377000) * 65536 / 6500;
    return N >> 8;
}

/*
* The frequency cannot be lower than 377000 as specified in section 6.3.2. The frequency range related
* to Fine Resolution Channel Scheme CNM.CM = 1 is from 389.5MHz to 510MHz
*/
uint8_t TransceiverTask::calculatePllChannelNumber09(uint32_t frequency) {
    uint32_t N = (frequency - 377000) * 65536 / 6500;
    return N & 0xFF;
}

void  TransceiverTask::directModConfigAndPreEmphasisFilter(bool enableDM, bool enablePE, bool recommended){
    if(enableDM){
        transceiver.set_direct_modulation(RF09, enableDM, error);
        transceiver.spi_write_8(BBC0_FSKDM, 0b00000001, error);
        if(enablePE){
            transceiver.spi_write_8(BBC0_FSKDM, 0b00000011, error);
            if(recommended){
                transceiver.spi_write_8(BBC0_FSKPE0, 0x2, error);
                transceiver.spi_write_8(BBC0_FSKPE1, 0x3, error);
                transceiver.spi_write_8(BBC0_FSKPE2, 0xFC, error);
            }
        }
    }
}

void TransceiverTask::txSRandTxFilter() {
    AT86RF215::RegisterAddress  regtxdfe;
    regtxdfe = AT86RF215::RF09_TXDFE;

    uint8_t reg = transceiver.spi_read_8(regtxdfe,error);
    // RCUT CONFIG //
    transceiver.spi_write_8(regtxdfe, reg | (static_cast<uint8_t>(0x01) << 5), error);
    // SR CONFIG //
    reg = transceiver.spi_read_8(regtxdfe,error);
    transceiver.spi_write_8(regtxdfe, reg | (static_cast<uint8_t>(0xA)), error);
}

void TransceiverTask::txAnalogFrontEnd() {
    AT86RF215::RegisterAddress regtxcutc;
    regtxcutc = AT86RF215::RF09_TXCUTC;

    uint8_t reg = transceiver.spi_read_8(regtxcutc, error);
    // PARAMP Config
    transceiver.spi_write_8(regtxcutc, reg | (static_cast<uint8_t>(AT86RF215::PowerAmplifierRampTime::RF_PARAMP32U) << 6), error);
    // LPFCUT Config
    transceiver.spi_write_8(regtxcutc, reg | (static_cast<uint8_t>(AT86RF215::TransmitterCutOffFrequency::RF_FLC80KHZ)), error);
}

void TransceiverTask::modulationConfig(){
    // BT = 1 , MIDXS = 1, MIDX = 1, MOR = B-FSK
    transceiver.spi_write_8(BBC0_FSKC0, 86, error);
    directModConfigAndPreEmphasisFilter(true,false, false);
}

void TransceiverTask::setRFmode(uint8_t mode) {
    if(mode)
    {
        transceiver.set_state(AT86RF215::RF09, State::RF_TXPREP, error);
        vTaskDelay(pdMS_TO_TICKS(10));
        transceiver.set_state(AT86RF215::RF09, State::RF_RX, error);
        if (transceiver.get_state(AT86RF215::RF09, error) == (AT86RF215::State::RF_RX))
            LOG_DEBUG << " STATE = RX ";
    }
    else{
        transceiver.set_state(AT86RF215::RF09, State::RF_TX, error);
        transceiver.TransmitterFrameEnd_flag = true;
    }
}

void TransceiverTask::init_transceiver() {
    setConfiguration(calculatePllChannelFrequency09(FrequencyUHF), calculatePllChannelNumber09(FrequencyUHF));
    transceiver.chip_reset(error);
    transceiver.setup(error);
    txAnalogFrontEnd();
    txSRandTxFilter();
    modulationConfig();
    receiverConfig(true);

    uint8_t reg = transceiver.spi_read_8(AT86RF215::BBC0_PC, error);
    //  ENABLE TXSFCS (FCS autonomously calculated)
    transceiver.spi_write_8(AT86RF215::BBC0_PC, reg | (1 << 4), error);
}


void TransceiverTask::execute() {
    while (checkTheSPI() != 0){
        vTaskDelay(10);
    };

    init_transceiver();
    setRFmode(rfModes::RX);

    uint32_t ok_packets = 0, wrong_packets = 0, sent_packets = 0;
    uint8_t low_length_byte = 0;
    uint8_t high_length_byte = 0;
    uint16_t received_length = 0;
    uint8_t button_pressed_times = 0;

    uint16_t currentPacketLength = MaxPacketLength;
    PacketType packet = createRandomPacket(MaxPacketLength);


    while(true) {
        if(button_flag)
        {
            button_pressed_times++;
            LOG_DEBUG << "button pressed" ;
            button_flag = 0;
            if(button_pressed_times == 3)
            {
                button_pressed_times = 0;
                if(txrx)
                {
                    txrx = 0;
                    LOG_DEBUG << "TX MODE" ;
                    transceiver.TransmitterFrameEnd_flag = true;
                    init_transceiver();
                }
                else{
                    txrx = 1;
                    init_transceiver();
                    setRFmode(rfModes::RX);
                }
            }
        }
        if(transceiverTask->txrx && transceiver.ReceiverFrameEnd_flag)
        {
            transceiver.ReceiverFrameEnd_flag = false;
            // Filtering the received packets //
            low_length_byte = transceiver.spi_read_8(AT86RF215::BBC0_RXFLL,error);
            high_length_byte = transceiver.spi_read_8(AT86RF215::BBC0_RXFLH,error);
            received_length = (high_length_byte << 8) | low_length_byte ;

            if(transceiver.spi_read_8(AT86RF215::BBC0_FBRXS, error) == packet_id && received_length == MaxPacketLength){
                ok_packets++;
                LOG_DEBUG << "PACKET RECEPTION OK, " << ok_packets ;
            }
            else{
                wrong_packets++;
                LOG_DEBUG << "ERROR IN PACKET RECEPTION, " <<  wrong_packets;
                for(int i = 0 ; i < MaxPacketLength; i++)
                    LOG_DEBUG << transceiver.spi_read_8((AT86RF215::BBC0_FBRXS) + i, error) ;
            }

            transceiver.set_state(AT86RF215::RF09, State::RF_TXPREP, error);
            vTaskDelay(pdMS_TO_TICKS(10));
            transceiver.set_state(AT86RF215::RF09, State::RF_RX, error);
        }
        if(transceiverTask->txrx == false && transceiver.TransmitterFrameEnd_flag)
        {
            sent_packets++;
            transceiver.transmitBasebandPacketsTx(AT86RF215::RF09, packet.data(), currentPacketLength, error);
            vTaskDelay(pdMS_TO_TICKS(200));
            setRFmode(rfModes::TX);
            transceiver.TransmitterFrameEnd_flag = false;
            LOG_DEBUG << "PACKET IS SENT " << sent_packets ;
        }
    }
}
