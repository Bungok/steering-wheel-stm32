/**
* @file lcd_control.c
* @brief Biblioteka do kontroli wyswietlacza Nextion Enhanced NX3224K028
* @author Piotr Durakiewicz
* @date 28.12.2020
* @todo
* @bug
* @copyright 2020 HYDROGREEN TEAM
*/

#include "lcd_control.h"
#include "Nextion_Enhanced_NX3224K028.h"
#include "Nextion_Enhanced_Expansion_Board.h"
#include "buttons.h"
#include "rs485.h"
#include "timers.h"
#include "watchdog.h"
#include "hydrogreen.h"

#define USE_EXPANSION_BOARD 			0

// ******************************************************************************************************************************************************** //

static uint16_t cntTickInitPage;		///< Zmienna odemierzajaca czas przez ktory ma zostac wyswietlana strona startowa w trakcie inicjalizacji
static uint16_t cntTickMode1Page;		///< Zmienna odmierzajaca czas co ktory maja zostac zaktualizowane wartosci na LCD w trybie MODE1_PAGE
static uint16_t cntTickEmPage;			///< Zmienna odmierzajaca czas w trybie "EM_PAGE"
#if USE_EXPANSION_BOARD == 1
static uint16_t cntTickLeakPage;		///< Zmienna odmierzajaca czas w trybie "LEAK_PAGE"
#endif
static uint16_t cntTickDevReset;		///< Zmienna odmierzajaca czas do resetu urzadzenia

static uint8_t mainStepFsm;			///< FSM funkcji lcd_control_step()
static uint8_t initFsm;				///< FSM funkcji initPage()
static uint8_t initCplt;			///< Flaga informujaca o zakonczeniu inicjaliacji (jezeli 1 = inicjalizacja zakonczona)
static uint8_t mode1FsmLowVal;			///< FSM funkcji mode1Page(), dla wartosci wymagajacych czestszego odswiezania
static uint8_t mode1FsmHighVal;			///< FSM funkcji mode1Page(), dla wartosci ktore moga byc aktualizowane rzadziej

// ******************************************************************************************************************************************************** //

/**
* @enum MAIN_MENU_FSM
* @brief Typ wyliczniowy zawierajacy ekrany LCD
*/
typedef enum
{
  INIT_PAGE,
  MODE1_PAGE,
  LEAK_PAGE,
  EM_PAGE
} MAIN_MENU_FSM;

// ******************************************************************************************************************************************************** //

void lcd_control_step(void);
static void initPage(void);
static void mode1Page(void);
static void resetAllCntAndFsmState(void);
static uint8_t emPage(void);
#if USE_EXPANSION_BOARD == 1
static void leakPage(void);
#endif
static uint8_t choosePage(void);
static uint8_t prevMs = 0;
static uint8_t prevS = 0;
static uint8_t prevM = 0;

// ******************************************************************************************************************************************************** //

/**
* @fn lcd_control_step(void)
* @brief Glowna funkcja obslugujaca wyswietlacz, powinna zostac wywolana wewnatrz hydrogreen_step1kHz()
*/
void lcd_control_step(void)
{
  if (initCplt) choosePage();	//Zmiana strony jest mozliwa dopiero po zakonczeniu inicjalizacji LCD (initCplt musi wynosic 1)

  switch (mainStepFsm)
  {
    case INIT_PAGE:
      initPage();
      break;

    case MODE1_PAGE:
      mode1Page();
      break;

    case LEAK_PAGE:
#if USE_EXPANSION_BOARD == 1
      leakPage();
#endif
      break;

    case EM_PAGE:
      emPage();
      break;

    default:
      break;
  }
}

/**
* @fn initPage(void)
* @brief Inicjalizacja wyswietlacza
*/
static void initPage(void)
{
  cntTickInitPage++;

  switch (initFsm)
  {
    //Zresetuj wyswietlacz
    case 0:
      if (Nextion_Enhanced_NX3224K028_deviceReset()) initFsm++;
      break;

    //Odczekaj 150ms (jest to czas inicjalizacji wyswietlacza)
    case 1:
      if (cntTickInitPage > 150 * PERIOD_1MS)
	{
	  cntTickInitPage = 0;
	  initFsm++;
	}
      break;

      //Narysuj prostokat wyznaczajacy krawedzie LCD
    case 2:
      if (Nextion_Enhanced_NX3224K028_drawRectangle(0, 0, 320, 240, (const uint8_t *)"GRAY"))
	{
#if USE_EXPANSION_BOARD == 1
	  initFsm++;
#else
	  initFsm = 11;
#endif
	}
      break;

#if USE_EXPANSION_BOARD == 1
    case 3:
      if (Nextion_Enhanced_Expansion_Board_configureGPIO(0, 2, 0)) initFsm++;
      break;

    case 4:
      if (Nextion_Enhanced_Expansion_Board_configureGPIO(1, 2, 0)) initFsm++;
      break;

    case 5:
      if (Nextion_Enhanced_Expansion_Board_configureGPIO(2, 2, 0)) initFsm++;
      break;

    case 6:
      if (Nextion_Enhanced_Expansion_Board_configureGPIO(3, 2, 0)) initFsm++;
      break;

    case 7:
      if (Nextion_Enhanced_Expansion_Board_configureGPIO(4, 2, 0)) initFsm++;
      break;

    case 8:
      if (Nextion_Enhanced_Expansion_Board_configureGPIO(5, 2, 0)) initFsm++;
      break;

    case 9:
      if (Nextion_Enhanced_Expansion_Board_configureGPIO(6, 2, 0)) initFsm++;
      break;

    case 10:
      if (Nextion_Enhanced_Expansion_Board_configureGPIO(7, 2, 0)) initFsm++;
      break;
#endif

      //Wyswietlaj przez 3 sekundy strone startowa
    case 11:
      if (cntTickInitPage >= 3 * PERIOD_1S)
	{
	  cntTickInitPage = 0;
	  initFsm++;
	}
      break;

    case 12:
      //Sprawdz czy wykryto wyciek wodoru
      if (RS485_RX_VERIFIED_DATA.h2SensorDigitalPin == 1)
	{
	  //Wyciek wodoru wykryty, przejdz do strony LEAK_PAGE
	  if (Nextion_Enhanced_NX3224K028_loadNewPage(3))
	    {
	      mainStepFsm = LEAK_PAGE;
	      initCplt = 1;
	      cntTickInitPage = 0;
	    }
	  break;
	}

      //Sprawdz czy przycisk bezpieczenstwa nie jest wduszony
      if (RS485_RX_VERIFIED_DATA.emergencyButton == 1)
	{
	  //Przycisk bezpieczenstwa jest wcisniety, przejdz do strony EM_PAGE
	  if (Nextion_Enhanced_NX3224K028_loadNewPage(4))
	    {
	      mainStepFsm = EM_PAGE;
	      initCplt = 1;
	      cntTickInitPage = 0;
	    }
	  break;
	}

      // Nie spelniono zadnej z powyzszych opcji, przejdz do domyslnego MODE1_PAGE
      if (Nextion_Enhanced_NX3224K028_loadNewPage(1))
	{
	  mainStepFsm = MODE1_PAGE;
	  initCplt = 1;
	  cntTickInitPage = 0;
	}
      break;

    default:
      break;
  }
}

/**
* @fn mode1Page(void)
* @brief Wyswietlanie informacji na LCD w trybie MODE_1
*/
static void mode1Page(void)
{
  cntTickMode1Page++;

  //Sprawdz czy czas miedzy aktualizacjami minal
  if (cntTickMode1Page >= 5 * PERIOD_1MS)
    {
      cntTickMode1Page = 0;

      //Wyswietlaj w pierwszej kolejnosci wartosci krytyczne (m.in paski postepu wymagajace czestego odswiezania)
      switch (mode1FsmHighVal)
      {

	  //Pasek postepu predkosc chwilowa
	case 0:
	  if (Nextion_Enhanced_NX3224K028_writeValueToProgressBar((const uint8_t*) "SB", RS485_RX_VERIFIED_DATA.interimSpeed, 50)) mode1FsmHighVal++;
	  break;

	  //Czas okrazenia (milisekundy)
	case 1:
	  if (Nextion_Enhanced_NX3224K028_writeNumberToControl((const uint8_t*) "ms", RS485_RX_VERIFIED_DATA.laptime_miliseconds.value))
	    {
#if USE_EXPANSION_BOARD == 1
	      mode1FsmHighVal++;
#else
	      mode1FsmHighVal = 9;
#endif
	    }
	  break;
	  //delta okrazenia(milisekundy)
	case 2:
		if (Nextion_Enhanced_NX3224K028_writeNumberToControl((const uint8_t*) "msd", RS485_RX_VERIFIED_DATA.delta_laptime_miliseconds.value))
		{
			mode1FsmHighVal++;
		}
		break;

#if USE_EXPANSION_BOARD == 1
	    case 3:
	      if (BUTTONS.speedReset == 1)
		{
		  if (Nextion_Enhanced_Expansion_Board_pinState(7, 1)) mode1FsmHighVal++;
		}
	      else
		{
		  if (Nextion_Enhanced_Expansion_Board_pinState(7, 0)) mode1FsmHighVal++;
		}
	      break;
#endif

	  //Wyswietlaj wartosci nie wymagajace tak czestego odswiezania
	case 4:
	  mode1FsmHighVal = 0;

	  switch (mode1FsmLowVal)
	    {

	      //Predkosc chwilowa
	    case 1:
	      if (Nextion_Enhanced_NX3224K028_writeNumberToControl((const uint8_t*) "V", RS485_RX_VERIFIED_DATA.interimSpeed)) mode1FsmLowVal++;
	      break;

	      //Czas okrazenia (minuty)
	    case 2:
	      if (Nextion_Enhanced_NX3224K028_writeNumberToControl((const uint8_t*) "mi", RS485_RX_VERIFIED_DATA.laptime_minutes.value))mode1FsmLowVal++;
	      break;

	      //Delta okrazenia (minuty)
	    case 3:
	    	if (Nextion_Enhanced_NX3224K028_writeNumberToControl((const uint8_t*) "mid", RS485_RX_VERIFIED_DATA.delta_laptime_minutes.value))mode1FsmLowVal++;
	    	break;

	      //Czas okrazenia (sekundy)
	    case 4:
	      if (Nextion_Enhanced_NX3224K028_writeNumberToControl((const uint8_t*) "sec", RS485_RX_VERIFIED_DATA.laptime_seconds)) mode1FsmLowVal++;
	      break;

	      //Delta okrazenia (sekundy)
	    case 5:
	      if (Nextion_Enhanced_NX3224K028_writeNumberToControl((const uint8_t*) "secd", RS485_RX_VERIFIED_DATA.delta_laptime_seconds)) mode1FsmLowVal++;
	      break;

	      //Moc calkowita
	    case 6:
	    	if (Nextion_Enhanced_NX3224K028_writeFltToControl((const uint8_t*) "TP", RS485_RX_VERIFIED_DATA.TOTAL_POWER.value)) mode1FsmLowVal++;
	    	break;

	    	//zuzycie wodoru
	    case 7:
	    	if (Nextion_Enhanced_NX3224K028_writeFltToControl((const uint8_t*) "hydusg", RS485_RX_VERIFIED_DATA.hydrogen_usage.value)) mode1FsmLowVal++;
	    	break;

	      //Sygnalizuj stan przycisku SUPPLY_BUTTON w postaci kolorowej obwodki na wokol ekranu (jezeli czerwona - zasilanie jest wylaczone)
	    case 8:
	      if (BUTTONS.powerSupply == 1)
		{
		  if (Nextion_Enhanced_NX3224K028_drawRectangle(0, 0, 320, 240, (const uint8_t*) "GRAY")) mode1FsmLowVal++;
		}
	      else
		{
		  if (Nextion_Enhanced_NX3224K028_drawRectangle(0, 0, 320, 240, (const uint8_t*) "RED")) mode1FsmLowVal++;
		}
	      break;

	    default:
	      break;

	  break;
	    }

	default:
	  break;
	}
    }
}

#if USE_EXPANSION_BOARD == 1
/**
* @fn leakPage(void)
* @brief Wyswietlanie informacji na LCD w trybie LEAK_PAGE
*/
static void leakPage(void)
{
  cntTickLeakPage++;

  if ( (cntTickLeakPage >= 100 * PERIOD_1MS) && (cntTickLeakPage < 150 * PERIOD_1MS) )
    {
      Nextion_Enhanced_Expansion_Board_pinState(7, 1);
    }
  else if (cntTickLeakPage >= 150 * PERIOD_1MS)
    {
      Nextion_Enhanced_Expansion_Board_pinState(7, 0);
      cntTickLeakPage = 0;
    }
}
#endif

/**
* @fn emPage(void)
* @brief Wyswietlanie informacji na LCD w trybie EM_PAGE
*/
static uint8_t emPage(void)
{
  cntTickEmPage++;

  if ( (cntTickEmPage >= PERIOD_1S) && (cntTickEmPage < 2 * PERIOD_1S) )
    {
      Nextion_Enhanced_NX3224K028_writeValueToProgressBar((const uint8_t *)"em_button", 100, 100);

      return 0;
    }
  else if (cntTickEmPage >= 2 * PERIOD_1S)
    {
      Nextion_Enhanced_NX3224K028_writeValueToProgressBar((const uint8_t *)"em_button", 0, 100);
      cntTickEmPage = 0;
      return 1;
    }

  return 0;
}

/**
* @fn choosePage(void)
* @brief Wybor aktualnie wyswietlanej strony na LCD
*/
static uint8_t choosePage(void)
{
  //Sprawdz czy wykryto wyciek wodoru
  if ( (RS485_RX_VERIFIED_DATA.h2SensorDigitalPin == 1) && mainStepFsm != LEAK_PAGE )
    {
      resetAllCntAndFsmState();
      Nextion_Enhanced_NX3224K028_loadNewPage(3);
      mainStepFsm = LEAK_PAGE;

      return 1;
    }
  //Sprawdz czy
  else if ( (RS485_RX_VERIFIED_DATA.h2SensorDigitalPin != 1) && (RS485_RX_VERIFIED_DATA.emergencyButton != 1)
      && mainStepFsm == LEAK_PAGE )
    {
      resetAllCntAndFsmState();
      Nextion_Enhanced_NX3224K028_loadNewPage(1);
      mainStepFsm = MODE1_PAGE;

      return 1;
    }
  //Jezeli nie wykryto wycieku wodoru a przycisk bezpieczenstwa jest wcisniety
  else if ( (RS485_RX_VERIFIED_DATA.emergencyButton == 1) &&  (RS485_RX_VERIFIED_DATA.h2SensorDigitalPin == 0) && mainStepFsm != EM_PAGE  )
    {
      resetAllCntAndFsmState();
      Nextion_Enhanced_NX3224K028_loadNewPage(4);
      mainStepFsm = EM_PAGE;

      return 1;
    }
  else if ( (RS485_RX_VERIFIED_DATA.emergencyButton != 1) && (RS485_RX_VERIFIED_DATA.h2SensorDigitalPin != 1)
      && mainStepFsm == EM_PAGE  && mainStepFsm != LEAK_PAGE)
    {
      resetAllCntAndFsmState();
      Nextion_Enhanced_NX3224K028_loadNewPage(1);
      mainStepFsm = MODE1_PAGE;

      return 1;
    }
  //Sprawdz czy przycisk mode1 jest wcisniety
  else if ( (BUTTONS.mode1 == 1) && (BUTTONS.mode2 == 0) && (mainStepFsm != MODE1_PAGE) &&
      (RS485_RX_VERIFIED_DATA.h2SensorDigitalPin != 1) && (RS485_RX_VERIFIED_DATA.emergencyButton != 1) )
    {
      resetAllCntAndFsmState();
      Nextion_Enhanced_NX3224K028_loadNewPage(1);
      mainStepFsm = MODE1_PAGE;

      return 1;
    }
  //Sprawdz czy przyciski mode1 oraz mode2 sa wcisniete jednoczesne
  else if ((BUTTONS.mode1 == 1) && (BUTTONS.mode2 == 1))
    {
      cntTickDevReset++;

      //Odczekaj 5 sekund, nastepnie ponownie zainicjalizuj wyswietlacz
      if (cntTickDevReset >= 5 * PERIOD_1S)
	{
	  resetAllCntAndFsmState();
	  initCplt = 0;
	  mainStepFsm = INIT_PAGE;

	  return 1;
	}
    }

  return 0;
}

/**
* @fn resetAllCntAndFsmState(void)
* @brief Funkcja resetujaca wszystkie zmienne odmierzajaca czas w bibliotece
*/
static void resetAllCntAndFsmState(void)
{
  cntTickInitPage = 0;
  cntTickMode1Page = 0;
  cntTickMode2Page = 0;
  cntTickEmPage = 0;
#if USE_EXPANSION_BOARD == 1
  cntTickLeakPage = 0;
#endif
  cntTickDevReset = 0;

  initFsm = 0;
  mode1FsmLowVal = 0;
  mode1FsmHighVal = 0;
  mode2Fsm = 0;
}
