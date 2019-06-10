/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//	ZMIENNE DO POBIERANIA LOKALIZACJI
uint16_t sizeReceiveUART = 1;
uint8_t receiveUART[1];	//kontener na dane z modu�u GPS
uint8_t input[200];
uint16_t size = 7;
uint8_t signal[7]={'$', 'G', 'P', 'G', 'L', 'L', '\0'};	//ci�g znak�w, kt�ry szukamy w danych z modu�u GPS
uint8_t geomsg[80];			//kontener na wiadomo�c z lokalizacj�
uint8_t space[1]={' '};
uint8_t end[1]={26};		//zako�czenie wiadomo�ci
uint8_t LAT[15];			//szeroko�c geograficzna
uint8_t LON[15];			//d�ugo�c geograficzna
uint8_t charac='1';
int i=0;					//zmienne do iteracji po znakach linii danych
int pos=0;
int k=0;
int licznik = -1;


//	KOMENDY AT DO KONFIGURACJI MODU�U GSM, WYSY�ANIA I ODBIERANIA WIADOMO�CI
uint8_t ate[5] = {'A', 'T', 'E', '0', '\r'};																		//komenda AT wy��czaj�ca echo
uint8_t at[3] = {'A', 'T', '\r'};																					//komenda AT sprawdzaj�ca po��czenie - modu� odpowiada OK
uint8_t cmgf[10]={'A','T','+','C','M','G','F','=','1', '\r'};														//komenda AT ustawiaj�ca tryb wysy�ania i odbierania wiadomo�ci
uint8_t cmgr[10]={'A','T','+','C','M','G','R','=','1', '\r'};														//komenda AT sprawdzaj�ca, czy na pozycji 1 znajduje si� nowa wiadomo�c
uint8_t cnmi[18]={'A','T','+','C','N','M','I','=','1',',','1',',','0',',','0',',','0', '\r'};						//komenda AT z ustawieniami powiadomie� o wiadomo�ciach SMS
uint8_t cpms[23]={'A','T','+','C','P','M','S','=','"','S','M','"',',','"','S','M','"',',','"','S','M','"', '\r'};	//komenda AT ustawiaj�ca zapis wiadomo�ci w pami�ci modu�u
uint8_t cmgd[12]={'A','T','+','C','M','G','D','=','1',',','4', '\r'};												//komenda AT usuwaj�ca wszystkie wiadomo�ci z pami�ci modu�u
uint8_t cmgs[23]={'A','T','+','C','M','G','S','=','"','+','4','8','7','8','0','1','2','2','3','1','4','"','\r'};	//komenda AT inicjalizuj�ca wysy�anie SMS na numer u�ytkownika
uint8_t receiveREQUEST[100];	//kontener na odpowiedzi modu�u



//********************************************* FLAGA GOOD *****************************************
int good = 0;

	/*	Flaga, kt�ra "zarz�dza" ca�ym procesem odbierania i wysy�ania wiadomo�ci.
		Przybiera 5 warto�ci:
			0 = pocz�tkowa warto�c przed konfiguracj� modu�u GMS
			1 = po konfiguracji modu�u GSM czekamy na wiadomo�c od u�ytkownika
			2 = otrzymali�my wiadomo�c, pobieramy lokalizacj�
			3 = pobrali�my lokalizacj�, mo�na przygotowac wiadomosc do wys�ania
			4 = wiadomo�c z lokalizacj� jest gotowa do wys�ania
			Struktura dzia�ania procesu to		0 -> 1 -> 2 -> 3 -> 4 -> 1 -> 2 -> 3 -> 4 -> 1 ->...
	*/

//***************************************************************************************************

int valueon=1;
int valueoff=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
   if(huart->Instance == USART3){
	   //****************************************ODBI�R DANYCH Z MODU�U GPS, WYCI�GANIE Z DANYCH D�UGO�CI I SZEROKO�CI GEOGRAFICZNEJ******************************************

	   //	1.	Sprawdzamy, czy komunikat zaczyna sie od $GPGLL

	  if(good==2){
	   if(receiveUART[0]=='$'){						//wykryto now� lini�
		   if(licznik==-1){							//czy kontener jest pusty?
			   input[0]=receiveUART[0];				//wsadzamy znak do kontenera
			   licznik=1;							//mo�na pobierac dalej
		   }
		   else good=3;								//zako�czenie pobierania linii zaczynaj�cej si� od $GPGLL, mo�emy przej�c do przygotowywania wiadomo�ci do wys�ania
	   }
	   else if(licznik>0){							//wsadzamy kolejne znaki po $ do kontenera
		   input[licznik]=receiveUART[0];
		   licznik++;

		   if(licznik==6){							//pobrali�my 6 znak�w
		   		   if(strcmp(input, signal)!=0){	//sprawdzamy, czy nasze 6 znak�w to $GPGLL
		   			   licznik=-1;					//je�li nie, zaczynamy od nowa, je�li tak, to pobieramy dalej a� napotkamy znak $ (nowa linia)
		   		   }
		   	   }

	   }
	  }

	  //	2. Mamy lini� kt�ra nas interesuje, wyci�gamy z niej szeroko�c i dlugo�c geograficzn� i stworzyc wiadomo�c do wys�ania

	  if(good==3){

		  charac=input[7];		// GPS wysy�a dane nawet jesli nie ma ��czno�ci z satelit� - 8. znak takiej linii danych jest wtedy przecinkiem
		  if(charac==','){good=2;}	//data invalid, wracamy do punktu 1.

		  else{		//data valid, wyci�gamy z linii danych szeroko�c geograficzn� iteruj�c po znakach
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, valueon);
			  for(i=7; ; i++){
				if(input[i]==','){
					LAT[k]=input[i+1];
					pos=i+3;
				break;			//po naptkaniu przecinka koniec pobierania szerokosci
			  }
				LAT[k]=input[i];	//pobieranie szeroko�ci
				k++;
		  }


		  	//	wyci�gamy z linii d�ugo�c geograficzn� w taki sam spos�b
		  	k=0;
		  	for(i=pos; ; i++){
		  		if(input[i]==','){
		  		LON[k]=input[i+1];
		  		break;
		  	}
		  		LON[k]=input[i];
		  		k++;
		  	}
		  		k=0;

		  // ��czymy szeroko�c i d�ugo�c w jedn� wiadomo�c
		  strcat(geomsg, LAT);
		  strcat(geomsg, space);
		  strcat(geomsg, LON);
		  strcat(geomsg, end);
		  good=4;	// mamy gotow� wiadomo�c, przechodzimy do jej wys�ania

		 }
	  }



       HAL_UART_Receive_IT(&huart3, receiveUART, sizeReceiveUART);

   }
   	   // ********************************************************************************************************************************************************************




   	   // ******************************************************** ODBIERANIE WIADOMO�CI SMS *********************************************************************************
   else if(huart->Instance == USART2){

   	//sprawdzamy, czy modu� GSM otrzyma� now� wiadomo�c - na komend� AT+CMGR=1 odpowiada wtedy +CMGR: <dane o wiadomosci>

		if(good==1){
			if(receiveREQUEST[3]=='C' && receiveREQUEST[4]=='M' && receiveREQUEST[5]=='G' && receiveREQUEST[6]=='R'){
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, valueon);
				good=2;				//dostali�my wiadomo�c, mo�emy przej�c do odbierania lokalizacji
				HAL_UART_Transmit_IT(&huart2, cmgd, strlen(cmgd));		//usuwamy wiadomo�c
			}
		}

   }

   // ************************************************************************************************************************************************************************


}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit_IT(&huart2, ate, 5);				//wy��czamy echo
  HAL_Delay(200);
  HAL_UART_Receive_IT(&huart2, receiveREQUEST, 11);

  HAL_UART_Transmit_IT(&huart2, at, strlen(at));		//sprawdzamy po��czenie
  HAL_Delay(200);
  HAL_UART_Receive_IT(&huart2, receiveREQUEST, 6);

  HAL_UART_Transmit_IT(&huart2, cmgf, strlen(cmgf));	//ustawiamy tryb wysy�ania i odbierania wiadomo�ci
  HAL_Delay(200);
  HAL_UART_Receive_IT(&huart2, receiveREQUEST, 6);

  HAL_UART_Transmit_IT(&huart2, cnmi, strlen(cnmi));	//ustawiamy powiadomienia
  HAL_Delay(200);
  HAL_UART_Receive_IT(&huart2, receiveREQUEST, 6);

  HAL_UART_Transmit_IT(&huart2, cpms, strlen(cpms));	//ustawiamy zapisywanie wiadomo�ci
  HAL_UART_Receive_IT(&huart2, receiveREQUEST, 31);

  good=1;		//skonfigurowali�my modu� GSM, czekamy na wiadomo�c od u�ytkownika



  HAL_UART_Receive_IT(&huart3, receiveUART, sizeReceiveUART);	//zaczynamy pobierac dane od modu�u GPS -
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	//upewniamy si� wtedy, �e nie straci przez swoj� bezczynno�c po��czenia

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //********************************************************* WYSY�ANIE WIADOMO�CI *******************************************************************

	  	  // 1. Sprawdzamy, czy modu� GSM otrzyma� wiadomo�c - pro�b� o lokalizacj�
	  if(good==1){
		  HAL_UART_Transmit_IT(&huart2, cmgr, strlen(cmgr));
		  HAL_Delay(200);
		  HAL_UART_Receive_IT(&huart2, receiveREQUEST, 15);
		  HAL_Delay(500);
	  }

	  	  // 2. Po otrzymaniu pro�by o lokalizacj� i stworzeniu wiadomo�ci z lokalizacj� wysy�amy j� do u�ytkownika
	  if(good==4){
		  HAL_UART_Transmit_IT(&huart2, cmgs, strlen(cmgs));
		  		  HAL_Delay(200);
		  		  HAL_UART_Transmit_IT(&huart2, geomsg, strlen(geomsg));
		  		  HAL_Delay(200);

		  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, valueoff);
		  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, valueoff);

		  		  good=1;	//wracamy do pocz�tku - czekamy na kolejn� wiadomo�c od u�ytkownika
	  }

	  //****************************************************************************************************************************************************

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
