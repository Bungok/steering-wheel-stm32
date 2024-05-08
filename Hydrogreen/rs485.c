/**
 * @file rs485.c
 * @brief Biblioteka do obslugi komunikacji UART <-> RS485 <-> UART
 * @author Piotr Durakiewicz
 * @date 08.12.2020
 * @todo
 * @bug
 * @copyright 2020 HYDROGREEN TEAM
 */

#include "rs485.h"
#include "buttons.h"
#include "usart.h"
#include "crc.h"

// ******************************************************************************************************************************************************** //

#define UART_PORT_RS485 		huart2
#define TX_FRAME_LENGHT 		11					///< Dlugosc wysylanej ramki danych (z suma CRC)
#define RX_FRAME_LENGHT 		39					///< Dlugosc otrzymywanej ramki danych (z suma CRC)
#define SOT_BYTE			0x12					///< Bajt wskazujacy na poczatek ramki
#define EOT_BYTE			0x17					///< Bajt wskazujacy na koniec ramki
#define BUFFER_SIZE         128

// ******************************************************************************************************************************************************** //

static uint8_t dataFromRx[BUFFER_SIZE]; ///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
volatile static uint16_t posInRxTab;///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
volatile static uint8_t intRxCplt; ///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
static uint8_t dataToTx[TX_FRAME_LENGHT]; ///< Tablica w ktorej zawarta jest ramka danych do wyslania
static uint16_t posInTxTab;	///< Aktualna pozycja w tabeli wykorzystywanej do wysylania danych
uint8_t rs485_flt = RS485_NEW_DATA_TIMEOUT;	///< Zmienna przechowujaca aktualny kod bledu magistrali

uint8_t hasSynchronized = 0;
uint8_t oldSyncArray[4];
uint8_t syncArray[4];

// ******************************************************************************************************************************************************** //

/**
 * @struct RS485_BUFFER
 * @brief Struktura zawierajaca bufory wykorzystywane do transmisji danych
 */
typedef struct {
	uint8_t tx;
	uint8_t rx;
} RS485_BUFFER;
static RS485_BUFFER RS485_BUFF;

RS485_RECEIVED_VERIFIED_DATA RS485_RX_VERIFIED_DATA; ///< Struktura w ktorej zawarte sa SPRAWDZONE przychodzace dane

// ******************************************************************************************************************************************************** //

static void sendData(void);
static void receiveData(void);
static void prepareNewDataToSend(void);
static void processReceivedData(void);
static void resetActData(void);

// ******************************************************************************************************************************************************** //

/**
 * @fn rs485_init(void)
 * @brief Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void)
 */
void rs485_init(void) {
//	memset(dataFromRx, '\0', RX_FRAME_LENGHT);
//	memset(dataToTx, '\0', TX_FRAME_LENGHT);

	prepareNewDataToSend(); //Przygotuj nowy pakiet danych

//	//HAL_UART_Receive_DMA(&UART_PORT_RS485, &RS485_BUFF.rx, 1);
	HAL_UART_Receive_IT(&UART_PORT_RS485, &RS485_BUFF.rx, 1);
	//Ringbuf_Init();
}

/**
 * @fn rs485_step(void)
 * @brief Funkcja obslugujaca magistrale, umiescic wewnatrz hydrogreen_step(void)
 */
void rs485_step(void) {
	receiveData();
	sendData();

}

/**
 * @fn sendData(void)
 * @brief Funkcja ktorej zadaniem jest obsluga linii TX, powinna zostac umieszczona w wewnatrz rs485_step()
 */
static void sendData(void) {
	static uint16_t cntEndOfTxTick;	//Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

	//Sprawdz czy wyslano cala ramke danych
	if (posInTxTab < TX_FRAME_LENGHT) {
		//Nie, wysylaj dalej
		RS485_BUFF.tx = dataToTx[posInTxTab];

		//Na czas wysylania danych wylacz przerwania
		__disable_irq();
		HAL_UART_Transmit(&UART_PORT_RS485, &RS485_BUFF.tx, 1, HAL_MAX_DELAY);
		__enable_irq();

		posInTxTab++;
	} else if (cntEndOfTxTick < TX_FRAME_LENGHT) {
		//Cala ramka danych zostala wyslana, zacznij odliczac "czas przerwy" pomiedzy przeslaniem kolejnej ramki
		cntEndOfTxTick++;
	} else {
		//Przygotuj nowe dane do wysylki
		cntEndOfTxTick = 0;
		posInTxTab = 0;

		prepareNewDataToSend();
	}
}

/**
 * @fn receiveData(void)
 * @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step()
 */
static void receiveData(void) {
	//Nowe dane zostaly otrzymane, zeruj flage informujaca o zakonczeniu transmisji
	intRxCplt = 0;

	if (dataFromRx[RX_FRAME_LENGHT - 1] == EOT_BYTE) {
		//Na czas przetwarzania danych wylacz przerwania
		__disable_irq();

		//Czas minal, oznacza to koniec ramki
		posInRxTab = 0;

		//OBLICZ SUME KONTROLNA
		uint8_t crcSumOnMCU = HAL_CRC_Calculate(&hcrc, (uint32_t*) dataFromRx,
				(RX_FRAME_LENGHT - 2));

		//Sprawdz czy sumy kontrolne oraz bajt EOT (End Of Tranmission) sie zgadzaja
		//	if ((dataFromRx[RX_FRAME_LENGHT - 1] == EOT_BYTE)
		//			&& (crcSumOnMCU == dataFromRx[RX_FRAME_LENGHT - 2])) {
		processReceivedData();
		__enable_irq();
	}


}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//    // Check if UART2 trigger the Callback
//    if(huart->Instance == USART2)
//    {
//    	if (Size < RX_FRAME_LENGHT) {
//    		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dataFromRx, RX_FRAME_LENGHT);
//    		return;
//    	}
//
//    	if (dataFromRx[RX_FRAME_LENGHT - 2] != EOT_BYTE) {
//    		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dataFromRx, RX_FRAME_LENGHT);
//    		return;
//    	}
//
//    	uint8_t crcSumOnMCU = HAL_CRC_Calculate(&hcrc, (uint32_t*) dataFromRx,
//    			(RX_FRAME_LENGHT - 2));
//    	if (crcSumOnMCU != dataFromRx[RX_FRAME_LENGHT - 1]) {
//    		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dataFromRx, RX_FRAME_LENGHT);
//    		return;
//    	}
//
//    	processReceivedData();
//
//        // Start to listening again - IMPORTANT!
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dataFromRx, RX_FRAME_LENGHT);
//    }

//    uint16_t i, pos, start, length;
//    uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);
//
//    /* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated, but there is no new character during timeout */
//    if(dma_uart_rx.flag && currCNDTR == RX_FRAME_LENGHT)
//    {
//        dma_uart_rx.flag = 0;
//        return;
//    }
//
//    /* Determine start position in DMA buffer based on previous CNDTR value */
//    start = (dma_uart_rx.prevCNDTR < RX_FRAME_LENGHT) ? (RX_FRAME_LENGHT - dma_uart_rx.prevCNDTR) : 0;
//
//    if(dma_uart_rx.flag)    /* Timeout event */
//    {
//        /* Determine new data length based on previous DMA_CNDTR value:
//         *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
//         *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
//        */
//        length = (dma_uart_rx.prevCNDTR < RX_FRAME_LENGHT) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (RX_FRAME_LENGHT - currCNDTR);
//        dma_uart_rx.prevCNDTR = currCNDTR;
//        dma_uart_rx.flag = 0;
//    }
//    else                /* DMA Rx Complete event */
//    {
//        length = RX_FRAME_LENGHT - start;
//        dma_uart_rx.prevCNDTR = RX_FRAME_LENGHT;
//    }
//
//    /* Copy and Process new data */
//    for(i=0,pos=start; i<length; ++i,++pos)
//    {
//        dataFromRx[i] = dma_rx_buf[pos];
//    }
//
//    if (dataFromRx[RX_FRAME_LENGHT - 2] == EOT_BYTE) {
//		uint8_t crcSumOnMCU = HAL_CRC_Calculate(&hcrc, (uint32_t*) dataFromRx,
//				(RX_FRAME_LENGHT - 1));
//		if (crcSumOnMCU != dataFromRx[RX_FRAME_LENGHT - 1]) {
//			//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dataFromRx, RX_FRAME_LENGHT);
//			//return;
//		}
//		intRxCplt = 1;
//
//    }

//    int index = -1;
//    for (int i = 0; i < RX_FRAME_LENGHT; i++) {
//    	if (dataFromRx[i] == EOT_BYTE) {
//    		index = i;
//    		break;
//    	}
//    }
//    if (index == -1) {
//    	return;
//    }
//
//
//
//	if (dataFromRx[RX_FRAME_LENGHT - 2] != EOT_BYTE) {
//		//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dataFromRx, RX_FRAME_LENGHT);
//		return;
//	}

//	uint8_t crcSumOnMCU = HAL_CRC_Calculate(&hcrc, (uint32_t*) dataFromRx,
//			(RX_FRAME_LENGHT - 2));
//	if (crcSumOnMCU != dataFromRx[RX_FRAME_LENGHT - 1]) {
//		//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dataFromRx, RX_FRAME_LENGHT);
//		return;
//	}

//	processReceivedData();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//Zabezpieczenie przed wyjsciem poza zakres bufora
	if (posInRxTab > BUFFER_SIZE)
		posInRxTab = 0;

	//Przypisz otrzymany bajt do analizowanej tablicy
	dataFromRx[posInRxTab] = RS485_BUFF.rx;
	posInRxTab++;

	intRxCplt = 1;
	if (posInRxTab < RX_FRAME_LENGHT - 1) {
		HAL_UART_Receive_IT(&UART_PORT_RS485, &RS485_BUFF.rx, 1);
		return;
	}

	if (dataFromRx[posInRxTab] != EOT_BYTE) {
		HAL_UART_Receive_IT(&UART_PORT_RS485, &RS485_BUFF.rx, 1);
		return;
	}

	if (posInRxTab != RX_FRAME_LENGHT - 1) {
		posInRxTab = 0;
		memset(dataFromRx, '\0', BUFFER_SIZE);
		HAL_UART_Receive_IT(&UART_PORT_RS485, &RS485_BUFF.rx, 1);
		return;
	}
}

/**
 * @fn prepareNewDataToSend(void)
 * @brief Funkcja przygotowujaca dane do wysylki, wykorzystana wewnatrz sendData(void)
 */
static void prepareNewDataToSend(void) {
	uint8_t j = 0;

	///< Stany przyciskow
	dataToTx[j] = BUTTONS.halfGas;
	dataToTx[++j] = BUTTONS.fullGas;
	dataToTx[++j] = BUTTONS.horn;
	dataToTx[++j] = BUTTONS.speedReset;
	dataToTx[++j] = BUTTONS.powerSupply;
	dataToTx[++j] = BUTTONS.scClose;
	dataToTx[++j] = BUTTONS.fuelcellOff;
	dataToTx[++j] = BUTTONS.fuelcellPrepareToRace;
	dataToTx[++j] = BUTTONS.fuelcellRace;

	dataToTx[++j] = EOT_BYTE;

	//OBLICZ SUME KONTROLNA
	uint8_t calculatedCrcSumOnMCU = HAL_CRC_Calculate(&hcrc,
			(uint32_t*) dataToTx, (TX_FRAME_LENGHT - 2));

	//Wrzuc obliczona sume kontrolna na koniec wysylanej tablicy
	dataToTx[TX_FRAME_LENGHT - 1] = calculatedCrcSumOnMCU;
}

/**
 * @fn processReveivedData()
 * @brief Funkcja przypisujaca odebrane dane do zmiennych docelowych
 */
static void processReceivedData(void) {
	uint8_t i = 0;

	RS485_RX_VERIFIED_DATA.interimSpeed = dataFromRx[i];
	RS485_RX_VERIFIED_DATA.averageSpeed = dataFromRx[++i];

	for (uint8_t k = 0; k < 2; k++) {
		RS485_RX_VERIFIED_DATA.laptime_minutes.array[k] = dataFromRx[++i];
	}

	RS485_RX_VERIFIED_DATA.laptime_seconds = dataFromRx[++i];

	for (uint8_t k = 0; k < 2; k++) {
		RS485_RX_VERIFIED_DATA.laptime_miliseconds.array[k] = dataFromRx[++i];
	}
	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.FC_V.array[k] = dataFromRx[++i];
	}

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.FC_TEMP.array[k] = dataFromRx[++i];
	}

	for (uint8_t k = 0; k < 2; k++) {
		RS485_RX_VERIFIED_DATA.fcFanRPM.array[k] = dataFromRx[++i];
	}

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.CURRENT_SENSOR_FC_TO_SC.array[k] =
				dataFromRx[++i];
	}

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.CURRENT_SENSOR_SC_TO_MOTOR.array[k] =
				dataFromRx[++i];
	}

	RS485_RX_VERIFIED_DATA.fcToScMosfetPWM = dataFromRx[++i];
	RS485_RX_VERIFIED_DATA.motorPWM = dataFromRx[++i];

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.SC_V.array[k] = dataFromRx[++i];
	}

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.H2_SENSOR_V.array[k] = dataFromRx[++i];
	}

	RS485_RX_VERIFIED_DATA.h2SensorDigitalPin = dataFromRx[++i];
	RS485_RX_VERIFIED_DATA.emergencyButton = dataFromRx[++i];
}

/**
 * @fn resetActData
 * @brief Zerowanie zmiennych docelowych (odbywa sie m.in w przypadku zerwania transmisji)
 */
static void resetActData(void) {
	RS485_RX_VERIFIED_DATA.interimSpeed = 0;
	RS485_RX_VERIFIED_DATA.averageSpeed = 0;

	RS485_RX_VERIFIED_DATA.laptime_minutes.value = 0;
	RS485_RX_VERIFIED_DATA.laptime_seconds = 0;
	RS485_RX_VERIFIED_DATA.laptime_miliseconds.value = 0;

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.FC_V.array[k] = 0;
	}

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.FC_TEMP.array[k] = 0;
	}

	RS485_RX_VERIFIED_DATA.fcFanRPM.value = 0;

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.CURRENT_SENSOR_FC_TO_SC.array[k] = 0;
	}

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.CURRENT_SENSOR_SC_TO_MOTOR.array[k] = 0;
	}

	RS485_RX_VERIFIED_DATA.fcToScMosfetPWM = 0;
	RS485_RX_VERIFIED_DATA.motorPWM = 0;

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.SC_V.array[k] = 0;
	}

	for (uint8_t k = 0; k < 4; k++) {
		RS485_RX_VERIFIED_DATA.H2_SENSOR_V.array[k] = 0;
	}

	RS485_RX_VERIFIED_DATA.h2SensorDigitalPin = 0;
	RS485_RX_VERIFIED_DATA.emergencyButton = 0;
}
