#include "RF_RXTask.hpp"
#include "Logger.hpp"

using namespace AT86RF215;


uint16_t RF_RXTask::calculatePllChannelFrequency09(uint32_t frequency) {
    uint32_t N = (frequency - 377000) * 65536 / 6500;
    return N >> 8;
}

/*
* The frequency cannot be lower than 377000 as specified in section 6.3.2. The frequency range related
* to Fine Resolution Channel Scheme CNM.CM = 1 is from 389.5MHz to 510MHz
*/
uint8_t RF_RXTask::calculatePllChannelNumber09(uint32_t frequency) {
    uint32_t N = (frequency - 377000) * 65536 / 6500;
    return N & 0xFF;
}


uint8_t RF_RXTask::checkTheSPI() {
    uint8_t spi_error = 0;
    DevicePartNumber dpn = transceiver.get_part_number(error);
    etl::string<LOGGER_MAX_MESSAGE_SIZE> output;
    if(dpn == DevicePartNumber::AT86RF215)
    {
        output = "SPI IS OK";
        Logger::log(Logger::debug, output);
    }

    else{
        spi_error = 1;
        LOG_DEBUG << "SPI ERROR" ;
        transceiver.chip_reset(error);
    }
    return spi_error;
}

void RF_RXTask::execute() {

   transceiver.chip_reset(error);
   transceiver.setRXConfig(RXConfig::DefaultRXConfig());
   transceiver.setup(error);

    while (checkTheSPI() != 0){
        vTaskDelay(10);
    };

    LOG_DEBUG << "RX TASK" ;
    uint8_t  read_reg;
    LOG_DEBUG << "RECEIVER" ;
    read_reg = transceiver.spi_read_8(AT86RF215::RF09_RXBWC, error);
    LOG_DEBUG << "RF09_RXBWC = " << read_reg;
    read_reg = transceiver.spi_read_8(AT86RF215::RF09_RXDFE, error);
    LOG_DEBUG << "RF09_RXDFE = " << read_reg;
    read_reg = transceiver.spi_read_8(AT86RF215::RF09_AGCC, error);
    LOG_DEBUG << "RF09_AGCC = " << read_reg;
    read_reg = transceiver.spi_read_8(AT86RF215::RF09_AGCS, error);
    LOG_DEBUG << "RF09_AGCS = " << read_reg;
    int read_rssi = transceiver.int_spi_read_8(AT86RF215::RF09_RSSI,error);
    LOG_DEBUG << "RF09_RSSI = " << read_rssi;
    read_reg = transceiver.spi_read_8(AT86RF215::RF09_EDC, error);
    LOG_DEBUG << "RF09_EDC = " << read_reg;
    read_reg = transceiver.spi_read_8(AT86RF215::RF09_EDD, error);
    LOG_DEBUG << "RF09_EDD = " << read_reg;
    int read_edv = transceiver.int_spi_read_8(AT86RF215::RF09_EDV, error);
    LOG_DEBUG << "RF09_EDV = " << read_edv;
    LOG_DEBUG << "DELAY";
    vTaskDelay(3000);

}


