#include <pigpio.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>

#define EMBEDDED_IN_VM

#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FR_MSB 0x06
#define REG_FR_DIM 0x07
#define REG_FR_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_OCP 0x0B
#define REG_LNA 0x0C
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_FIFO_RX_BASE_ADDR 0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_RX_HEADER_CNT_VALUE_MSB 0x14
#define REG_RX_HEADER_CNT_VALUE_LSB 0x15
#define REG_RX_PACKET_CNT_VALUE_MSB 0x16
#define REG_RX_PACKET_CNT_VALUE_LSB 0x17
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_RSSI_VALUE 0x1B
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_MODEM_CONFIG_3 0x26
#define REG_PAYLOAD_LENGTH 0x22
#define REG_FIFO_RX_BYTE_ADDR 0x25
#define REG_PA_DAC 0x4d
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_TEMP 0x3c
#define REG_SYNC_WORD 0x39
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_DETECT_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37

#define TX_BASE_ADDR 0x00
#define RX_BASE_ADDR 0x00

#define LORA_MODE 0x80

#define SLEEP_MODE 0x00
#define STDBY_MODE 0x01
#define TX_MODE 0x03
#define RXCONT_MODE 0x05

#define IRQ_RXDONE 0x40
#define IRQ_TXDONE 0x08

typedef enum BandWidth{
    BW7_8 =0,
    BW10_4 = 1<<4,
    BW15_6 = 2<<4,
    BW20_8 = 3<<4,
    BW31_25 = 4<<4,
    BW41_7 = 5<<4,
    BW62_5 = 6<<4,
    BW125 = 7<<4,
    BW250 = 8<<4,
    BW500 = 9<<4,
} BandWidth;

typedef enum SpreadingFactor{
    SF7 = 7<<4,
    SF8 = 8<<4,
    SF9 = 9<<4,
    SF10 = 10<<4,
    SF11 = 11<<4,
    SF12 = 12<<4,
} SpreadingFactor;

typedef enum ErrorCodingRate{
    CR5 = 1<<1,
    CR6 = 2<<1,
    CR7 = 3<<1,
    CR8 = 4<<1,
} ErrorCodingRate;

typedef enum OutputPower{
    OP0 = 0,
    OP1 = 1,
    OP2 = 2,
    OP3 = 3,
    OP4 = 4,
    OP5 = 5,
    OP6 = 6,
    OP7 = 7,
    OP8 = 8,
    OP9 = 9,
    OP10 = 10,
    OP11 = 11,
    OP12 = 12,
    OP13 = 13,
    OP14 = 14,
    OP15 = 15,
    OP16 = 16,
    OP17 = 17,
    OP20 = 20,
} OutputPower;

typedef enum PowerAmplifireOutputPin{
    RFO = 0x70,
    PA_BOOST = 0xf0,
} PowerAmplifireOutputPin;

typedef enum LnaGain{
    G1 = 1,
    G2 = 2,
    G3 = 3,
    G4 = 4,
    G5 = 5,
    G6 = 6,
} LnaGain;

typedef struct{
    BandWidth bw;
    SpreadingFactor sf; //only from SF7 to SF12. SF6 not support yet.
    ErrorCodingRate ecr;
    double freq;// Frequency in Hz. Example 434000000
    unsigned char resetGpioN;//raspberry GPIO pin connected to RESET pin of LoRa chip
    unsigned char dio0GpioN;//raspberry GPIO pin connected to DIO0 pin of LoRa chip to detect TX and Rx done events.
    unsigned int preambleLen;
    _Bool lowDataRateOptimize;//Dont touch it sets automatically
    OutputPower outPower;
    PowerAmplifireOutputPin powerOutPin;//This chips has to outputs for signal "High power" and regular.
    int curRSSI;//Current ethereum RSSI
    unsigned char syncWord;
    LnaGain lnaGain;
    _Bool lnaBoost;//On/Off LNA boost
    _Bool AGC;// On/Off AGC. If AGC is on, LNAGain not used
    unsigned char OCP;//Over Current Protection. 0 to turn OFF. Else reduces current from 45mA to 240mA
    _Bool implicitHeader;// 1 - implicit header. 0 - Explicit header.
    unsigned char payloadLen;//Payload len that used in implicit mode. In Explicit header mode not used.
    _Bool CRC;//1 - add CRC data and checking. 0 - remove CRC data and checking
} Modem_cfg;

typedef struct{
    char *buf;
    unsigned char size;//Size of buffer. Used in Explicit header mode. 255 MAX size
    struct timeval last_time;
    double Tsym;
    double Tpkt;
    unsigned payloadSymbNb;
    void *userPtr;//user pointer passing to user callback
} txData;

typedef void (*UserTxDoneCallback)(txData *tx);

typedef struct{
    txData data;
    UserTxDoneCallback callback;
    pthread_t cbThread;
} LoRa_Tx;

typedef void (*txDoneISR)(int gpio_n, int level, uint32_t tick, void *userdata);

typedef struct{
    char *buf;
    unsigned char size;
    struct timeval last_time;
    float SNR;
    int RSSI;
    _Bool CRC;
    void *userPtr;//user pointer passing to user callback
} rxData;

typedef void (*UserRxDoneCallback)(rxData *rx);

typedef struct{
    rxData data;
    UserRxDoneCallback callback;
    pthread_t cbThread;
} LoRa_Rx;

typedef void (*rxDoneISR)(int gpio_n, int level, uint32_t tick, void *userdata);

typedef struct {
    unsigned char spiCS;//SPI CS number
    int spid;//SPI file descriptor
    Modem_cfg eth;//ethereum config structure
    LoRa_Rx rx;//rx structure
    LoRa_Tx tx;//tx structure
} LoRa_ctl;



static int lora_reg_write_byte(int spid, unsigned char reg, unsigned char byte){
    char rx[2], tx[2];
    tx[0]=(reg | 0x80);
    tx[1]=byte;
    
    rx[0]=0x00;
    rx[1]=0x00;
    
    return spiXfer(spid, tx, rx, 2);
}

static int lora_reg_read_bytes(int spid, unsigned char reg, char *buff, unsigned char size){
    int ret;
    char tx[257];
    char rx[257];
    
    memset(tx, '\0', 257);
    memset(rx, '\0', 257);
    memset(buff, '\0', size);
    tx[0]=reg;
    ret = spiXfer(spid, tx, rx, size+1);
    memcpy(buff, &rx[1], ret-1);
    return ret;
}

static int lora_reg_write_bytes(int spid, unsigned char reg, char *buff, unsigned char size){
    char tx[257];
    char rx[257];
    memset(tx, '\0', 257);
    memset(rx, '\0', 257);
    
    tx[0]=(reg | 0x80);
    memcpy(&tx[1], buff, size);
    return spiXfer(spid, tx, rx, size+1);
}

static unsigned char lora_reg_read_byte(int spid, unsigned char reg){
    int ret;
    char rx[2], tx[2];
    tx[0]=reg;
    tx[1]=0x00;
    
    rx[0]=0x00;
    rx[1]=0x00;
    
    ret = spiXfer(spid, tx, rx, 2);
    if(ret<0)
        return ret;
    
    if(ret<=1)
        return -1;
    
    return rx[1];
}





static void lora_set_ocp(int spid, unsigned char OCP){
    unsigned char OcpTrim;
    if(OCP == 0){//turn off OCP
        lora_reg_write_byte(spid, REG_OCP, (lora_reg_read_byte(spid, REG_OCP) & 0xdf));
    }
    else if(OCP > 0 && OCP <= 120){
        if(OCP < 50){OCP = 50;}
        
        OcpTrim = (OCP-45)/5 + 0x20;
        lora_reg_write_byte(spid, REG_OCP, OcpTrim);
    }
    else if(OCP > 120){
        if(OCP < 130){OCP = 130;}
        
        OcpTrim = (OCP+30)/10 + 0x20;
        lora_reg_write_byte(spid, REG_OCP, OcpTrim);
    }
}

static void lora_set_lna(int spid, LnaGain lnaGain, _Bool lnaBoost){
    lora_reg_write_byte(spid, REG_LNA,  ( (lnaGain << 5) + lnaBoost) );
}

static void lora_set_agc(int spid, _Bool AGC ){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_3, (AGC << 2));
}

static void lora_set_tx_power(int spid, OutputPower power, PowerAmplifireOutputPin pa_pin){
    lora_reg_write_byte(spid, REG_OCP, 0x1f);//Disable over current protection
    
    if(pa_pin == RFO){
        power = power >= OP15 ? OP15 : ( power <= OP0 ? OP0 : power);
        lora_reg_write_byte(spid, REG_PA_DAC, 0x84);//default val to +17dBm
        lora_reg_write_byte(spid, REG_PA_CONFIG, pa_pin | power);
        return;
    }else if(pa_pin == PA_BOOST){
        if(power == OP20){
            lora_reg_write_byte(spid, REG_PA_DAC, 0x87);//Max 20dBm power
            lora_reg_write_byte(spid, REG_PA_CONFIG, pa_pin | (power -2));
            return;
        }
        else{
            power = power >= OP17 ? OP17 : ( power <= OP2 ? OP2 : power);
            lora_reg_write_byte(spid, REG_PA_DAC, 0x84);//default val to +17dBm
            lora_reg_write_byte(spid, REG_PA_CONFIG, pa_pin | (power -2));
            return;
        }
    }
}

static void lora_set_dio_rx_mapping(int spid){
    lora_reg_write_byte(spid, REG_DIO_MAPPING_1, 0<<6);
}

static void lora_set_dio_tx_mapping(int spid){
    lora_reg_write_byte(spid, REG_DIO_MAPPING_1, 1<<6);
}



static void lora_set_txdone_dioISR(int gpio_n, txDoneISR func, LoRa_ctl *modem){
    gpioSetMode(gpio_n, PI_INPUT);
    gpioSetISRFuncEx(gpio_n, RISING_EDGE, 0, func, (void *)modem);
}

static void lora_remove_dioISR(int gpio_n){
    gpioSetISRFunc(gpio_n, RISING_EDGE, 0, NULL);
}

static void lora_set_lowdatarateoptimize_on(int spid){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_3, (lora_reg_read_byte(spid, REG_MODEM_CONFIG_3) & 0xf7) | (0x01<<3));
}

static void lora_set_lowdatarateoptimize_off(int spid){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_3, (lora_reg_read_byte(spid, REG_MODEM_CONFIG_3) & 0xf7));
}

static void lora_get_rssi_pkt(LoRa_ctl *modem){
    modem->rx.data.RSSI = lora_reg_read_byte(modem->spid, REG_PKT_RSSI_VALUE) - (modem->eth.freq < 779E6 ? 164 : 157);
}

static void lora_get_rssi_cur(LoRa_ctl *modem){
    modem->eth.curRSSI = lora_reg_read_byte(modem->spid, REG_RSSI_VALUE) - (modem->eth.freq < 779E6 ? 164 : 157);
}

static void lora_get_snr(LoRa_ctl *modem){
    modem->rx.data.SNR = ((char)lora_reg_read_byte(modem->spid, REG_PKT_SNR_VALUE))*0.25;
}

static void * startRxCallback(void *arg){
        LoRa_ctl *modem = (LoRa_ctl *)arg;
        modem->rx.callback(&modem->rx.data);
}


static void * startTxCallback(void *arg){
        LoRa_ctl *modem = (LoRa_ctl *)arg;
        modem->tx.callback(&modem->tx.data);
}

static void lora_set_sleep_mode(int spid){
    lora_reg_write_byte(spid, REG_OP_MODE, (lora_reg_read_byte(spid, REG_OP_MODE) & 0xf8) | SLEEP_MODE );
}


static void LoRa_sleep(LoRa_ctl *modem){
    lora_remove_dioISR(modem->eth.dio0GpioN);
    lora_set_sleep_mode(modem->spid);
}

static unsigned char lora_get_op_mode(int spid){
    return (lora_reg_read_byte(spid, REG_OP_MODE) & 0x07);
}

static void LoRa_calculate_packet_t(LoRa_ctl *modem){
    unsigned BW_VAL[10] = {7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};
    
    double Tsym, Tpreamle, Tpayload, Tpacket;
    unsigned payloadSymbNb;
    int tmpPoly;
    
    unsigned bw = BW_VAL[(modem->eth.bw>>4)];
    unsigned sf = modem->eth.sf>>4;
    unsigned char ecr = 4+(modem->eth.ecr/2);
    unsigned char payload;
    if(modem->eth.implicitHeader){
        payload = modem->eth.payloadLen;
    }
    else{
        payload = modem->tx.data.size;
    }
    
    Tsym = (pow(2, sf)/bw)*1000;
    modem->eth.lowDataRateOptimize = (Tsym > 16);
    
    Tpreamle = (modem->eth.preambleLen+4.25)*Tsym;
    tmpPoly = (8*payload - 4*sf + 28 + 16 - 20*modem->eth.implicitHeader);
    if(tmpPoly<0){
        tmpPoly=0;
    }
    payloadSymbNb = 8+ceil(((double)tmpPoly)/(4*(sf - 2*modem->eth.lowDataRateOptimize)))*ecr;
    Tpayload = payloadSymbNb*Tsym;
    Tpacket = Tpayload+Tpreamle;
    
    modem->tx.data.Tsym = Tsym;
    modem->tx.data.Tpkt = Tpacket;
    modem->tx.data.payloadSymbNb = payloadSymbNb;
}


static void lora_set_payload(int spid, unsigned char payload){
    lora_reg_write_byte(spid, REG_PAYLOAD_LENGTH, payload);
}

static void lora_set_addr_ptr(int spid, unsigned char addr){
    lora_reg_write_byte(spid, REG_FIFO_ADDR_PTR, addr);
}

static unsigned char lora_write_fifo(int spid, char *buf, unsigned char size){
    lora_set_addr_ptr(spid, TX_BASE_ADDR);
    lora_set_payload(spid, size);
    return lora_reg_write_bytes(spid, REG_FIFO, buf, size);
}

static void lora_set_satandby_mode(int spid){
    lora_reg_write_byte(spid, REG_OP_MODE, (lora_reg_read_byte(spid, REG_OP_MODE) & 0xf8) | STDBY_MODE );
}

static void lora_set_rxcont_mode(int spid){
    lora_reg_write_byte(spid, REG_OP_MODE, (lora_reg_read_byte(spid, REG_OP_MODE) & 0xf8) | RXCONT_MODE );
}

static void lora_set_tx_mode(int spid){
    lora_reg_write_byte(spid, REG_OP_MODE, (lora_reg_read_byte(spid, REG_OP_MODE) & 0xf8) | TX_MODE );
}
static void lora_set_syncw(int spid, unsigned char word){
    lora_reg_write_byte(spid, REG_SYNC_WORD, word);
}


static void lora_set_lora_mode(int spid){
    lora_set_sleep_mode(spid);
    lora_reg_write_byte(spid, REG_OP_MODE, (lora_reg_read_byte(spid, REG_OP_MODE) & 0x7f) | LORA_MODE );
}

static void lora_set_sf(int spid, SpreadingFactor sf){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_2, (lora_reg_read_byte(spid, REG_MODEM_CONFIG_2)& 0x0f) | sf );
}

static void lora_set_crc_on(int spid){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_2, (lora_reg_read_byte(spid, REG_MODEM_CONFIG_2)& 0xfb) | 0x01<<2 );
}

static void lora_set_crc_off(int spid){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_2, (lora_reg_read_byte(spid, REG_MODEM_CONFIG_2)& 0xfb));
}

static void lora_set_bandwidth(int spid, BandWidth bw){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_1, (lora_reg_read_byte(spid, REG_MODEM_CONFIG_1)& 0x0f) | bw );
}

static void lora_set_errorcr(int spid, ErrorCodingRate cr){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_1, (lora_reg_read_byte(spid, REG_MODEM_CONFIG_1)& 0xf1) | cr );
}

static void lora_set_explicit_header(int spid){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_1, lora_reg_read_byte(spid, REG_MODEM_CONFIG_1) & 0xfe );
}

static void lora_set_implicit_header(int spid){
    lora_reg_write_byte(spid, REG_MODEM_CONFIG_1, lora_reg_read_byte(spid, REG_MODEM_CONFIG_1) | 0x01 );
}

static void lora_set_freq(int spid, double freq){
    int frf, frf_revers=0;
    frf = (int)ceil((freq/(32000000.0))*524288);
    frf_revers += (int)((unsigned char)(frf>>0))<<16;
    frf_revers += (int)((unsigned char)(frf>>8))<<8;
    frf_revers += (int)((unsigned char)(frf>>16)<<0);
    lora_reg_write_bytes(spid, REG_FR_MSB, (char *)&frf_revers, 3);
}

static _Bool LoRa_check_conn(LoRa_ctl *modem){
    return (modem->eth.syncWord == lora_reg_read_byte(modem->spid, REG_SYNC_WORD));
}

static void lora_set_preamble(int spid, unsigned int preambleLen){
    if(preambleLen < 6){
        preambleLen = 6;
    }
    else if(preambleLen > 65535){
        preambleLen = 65535;
    }
    unsigned len_revers=0;
    len_revers += ((unsigned char)(preambleLen>>0))<<8;
    len_revers += ((unsigned char)(preambleLen>>8))<<0;
    lora_reg_write_bytes(spid, REG_PREAMBLE_MSB, (char *)&len_revers, 2);
}



static void lora_reset(unsigned char gpio_n){
    gpioSetMode(gpio_n, PI_OUTPUT);
    gpioWrite(gpio_n, 0);
    usleep(100);
    gpioWrite(gpio_n, 1);
    usleep(5000);
}

static void lora_reset_irq_flags(int spid){
    lora_reg_write_byte(spid, REG_IRQ_FLAGS, 0xff);
}



static unsigned char LoRa_get_op_mode(LoRa_ctl *modem){
    return lora_get_op_mode(modem->spid);
}


static void txDoneISRf(int gpio_n, int level, uint32_t tick, void *modemptr){
    LoRa_ctl *modem = (LoRa_ctl *)modemptr;
    if(lora_reg_read_byte(modem->spid, REG_IRQ_FLAGS) & IRQ_TXDONE){
        gettimeofday(&modem->tx.data.last_time, NULL);
        //lora_remove_dioISR(gpio_n);
        lora_reset_irq_flags(modem->spid);
        lora_set_satandby_mode(modem->spid);
        pthread_create(&(modem->tx.cbThread), NULL, startTxCallback, (void *)(modem));
    }
}



static int LoRa_begin(LoRa_ctl *modem){
    int cfg = gpioCfgGetInternals();
    cfg |= PI_CFG_NOSIGHANDLER;
    gpioCfgSetInternals(cfg);
    if (gpioInitialise() < 0)
    {
        printf("Pigpio init error\n");
        return -1;
    }
    
    lora_reset(modem->eth.resetGpioN);
    
    if( (modem->spid = spiOpen(modem->spiCS, 32000, 0))<0 )
        return modem->spid;
    
    lora_set_lora_mode(modem->spid);
    
    if(modem->eth.implicitHeader){
        lora_set_implicit_header(modem->spid);
        lora_set_payload(modem->spid, modem->eth.payloadLen);
    }
    else{
        lora_set_explicit_header(modem->spid);
    }
    
    lora_set_errorcr(modem->spid, modem->eth.ecr);
    lora_set_bandwidth(modem->spid, modem->eth.bw);
    lora_set_sf(modem->spid, modem->eth.sf);
    
    if(modem->eth.CRC){
        lora_set_crc_on(modem->spid);
    }
    else{
        lora_set_crc_off(modem->spid);
    }
    
    lora_set_tx_power(modem->spid, modem->eth.outPower, modem->eth.powerOutPin);
    lora_set_syncw(modem->spid, modem->eth.syncWord);
    lora_set_preamble(modem->spid, modem->eth.preambleLen);
    lora_set_agc(modem->spid, modem->eth.AGC);
    lora_set_lna(modem->spid, modem->eth.lnaGain, modem->eth.lnaBoost);
    lora_set_ocp(modem->spid, modem->eth.OCP);
    
    lora_reg_write_byte(modem->spid, REG_FIFO_TX_BASE_ADDR, TX_BASE_ADDR);
    lora_reg_write_byte(modem->spid, REG_FIFO_RX_BASE_ADDR, RX_BASE_ADDR);
    lora_reg_write_byte(modem->spid, REG_DETECT_OPTIMIZE, 0xc3);//LoRa Detection Optimize for SF > 6
    lora_reg_write_byte(modem->spid, REG_DETECTION_THRESHOLD, 0x0a);//DetectionThreshold for SF > 6
    
    lora_set_freq(modem->spid, modem->eth.freq);
    return modem->spid;
}

static void rxDoneISRf(int gpio_n, int level, uint32_t tick, void *modemptr){
    LoRa_ctl *modem = (LoRa_ctl *)modemptr;
    unsigned char rx_nb_bytes;
    if(lora_reg_read_byte(modem->spid, REG_IRQ_FLAGS) & IRQ_RXDONE){
        lora_reg_write_byte(modem->spid, REG_FIFO_ADDR_PTR, lora_reg_read_byte(modem->spid, REG_FIFO_RX_CURRENT_ADDR));
        
        if(modem->eth.implicitHeader){
            lora_reg_read_bytes(modem->spid, REG_FIFO, modem->rx.data.buf, modem->eth.payloadLen);
            modem->rx.data.size = modem->eth.payloadLen;
        }
        else{
            rx_nb_bytes = lora_reg_read_byte(modem->spid, REG_RX_NB_BYTES);
            lora_reg_read_bytes(modem->spid, REG_FIFO, modem->rx.data.buf, rx_nb_bytes);
            modem->rx.data.size = rx_nb_bytes;
        }
        modem->rx.data.CRC = (lora_reg_read_byte(modem->spid, REG_IRQ_FLAGS) & 0x20);
        lora_get_rssi_pkt(modem);
        lora_get_snr(modem);
        lora_reset_irq_flags(modem->spid);
        pthread_create(&(modem->rx.cbThread), NULL, startRxCallback, (void *)(modem));
    }
}

static void lora_set_rxdone_dioISR(int gpio_n, rxDoneISR func, LoRa_ctl *modem){
    gpioSetMode(gpio_n, PI_INPUT);
    gpioSetISRFuncEx(gpio_n, RISING_EDGE, 0, func, (void *)modem);
}

static void LoRa_receive(LoRa_ctl *modem){
    
    LoRa_calculate_packet_t(modem);
    if(modem->eth.lowDataRateOptimize){
        lora_set_lowdatarateoptimize_on(modem->spid);
    }
    else{
        lora_set_lowdatarateoptimize_off(modem->spid);
    }
    
    if(lora_get_op_mode(modem->spid) != STDBY_MODE){
        lora_set_satandby_mode(modem->spid);
    }
    lora_set_dio_rx_mapping(modem->spid);
    lora_set_rxdone_dioISR(modem->eth.dio0GpioN, rxDoneISRf, modem);
    lora_set_rxcont_mode(modem->spid);
}


static void LoRa_send(LoRa_ctl *modem){
    if(lora_get_op_mode(modem->spid) != STDBY_MODE){
        lora_set_satandby_mode(modem->spid);
    }
    LoRa_calculate_packet_t(modem);
    if(modem->eth.lowDataRateOptimize){
        lora_set_lowdatarateoptimize_on(modem->spid);
    }
    else{
        lora_set_lowdatarateoptimize_off(modem->spid);
    }
    
    if(modem->eth.implicitHeader){
        lora_write_fifo(modem->spid, modem->tx.data.buf, modem->eth.payloadLen);
    }
    else{
        lora_write_fifo(modem->spid, modem->tx.data.buf, modem->tx.data.size);
    }
    
    lora_set_dio_tx_mapping(modem->spid);
    lora_set_txdone_dioISR(modem->eth.dio0GpioN, txDoneISRf, modem);
    lora_set_tx_mode(modem->spid);
}

static void LoRa_stop_receive(LoRa_ctl *modem){
    lora_remove_dioISR(modem->eth.dio0GpioN);
    lora_set_satandby_mode(modem->spid);
}


static void LoRa_end(LoRa_ctl *modem){
    LoRa_stop_receive(modem);
    spiClose(modem->spid);
    modem->spid =-1;
}


static char rxbuf[1024];

extern void RX(rxData *rx);

static void initLora () {
     
    LoRa_ctl *modem = (LoRa_ctl*)malloc(sizeof(LoRa_ctl));
    modem->spiCS = 0;//Raspberry SPI CE pin number
    modem->rx.data.buf = rxbuf;
    modem->rx.callback = RX;
    modem->eth.preambleLen=8;
    modem->eth.bw = BW125;//Bandwidth 125KHz
    modem->eth.sf = SF7;//Spreading Factor 12
    modem->eth.ecr = CR5;//Error coding rate CR4/8
    modem->eth.freq = 433E6;// 434.8MHz
    modem->eth.resetGpioN = 4;//GPIO4 on lora RESET pi
    modem->eth.dio0GpioN = 17;//GPIO17 on lora DIO0 pin to control Rxdone and Txdone interrupts
    modem->eth.outPower = OP20;//Output power
    modem->eth.powerOutPin = PA_BOOST;//Power Amplifire pin
    modem->eth.AGC = 1;//Auto Gain Control
    modem->eth.OCP = 240;// 45 to 240 mA. 0 to turn off protection
    modem->eth.implicitHeader = 0;//Explicit header mode
    modem->eth.syncWord = 0x12;
    
    LoRa_begin(modem);
    LoRa_receive(modem);
}

 



