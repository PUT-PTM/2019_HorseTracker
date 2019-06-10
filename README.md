# :horse: 2019_HorseTracker

## 	:world_map: Overview
HorseTracker is a device that can send you its location through an SMS. 
All you have to do is just send it a "Localization" message and it will reply with precise
geographic coordinates to where it currently is. 
Place it anywhere - on your horse, your dog or your grandma with dementia who gets lost way too often these days.

## :pencil2: Description
Our device consists of 4 parts:
  - STM32F407VG microcontroller
  - dfRobot Gravity: A6 GSM & GPRS Module
  - u-blox GPS Neo-6m UART Module
  - 5V Powerbank
  
 The application was made in STM32CubeMX 4.1.0.0 and Eclipse.

## :runner: How to run
You don't need any addons in order to run the program. You do need to have STM32 installed.
After opening the main.c file just edit line 96:
> uint8_t cmgs[23]={'A','T','+','C','M','G','S','=','"','+','4','8','1','2','3','4','5','6','7','8','9','"','\r'};

ONLY edit the 11 characters after '+'. Put your phone number in there and... done! 
Your device's number is in line 70.

## :computer: How to compile
To compile the program just click compile!

## :rocket: Future improvements
Our ideas for future improvements are:
  - adding a speaker. In situations when you know you are close to your horse's coorditanes but you still can't see it,
  send a "Sound" SMS. The speaker will start beeping loudly, letting you know where your horse is hiding.
  - adding a list of phone numbers that you trust. Your and your horse's safety is our priority. We would like you to be able 
  to give your device a list of phone numbers that you trust. Be it your mom or your favourite
  teacher - your HorseTracker will only answer to their texts. Messages from unknown numbers will be ignored.
  - using a smaller MCU. HorseTracker 2.0 with smaller MCU would be so small you could use it on ponies!

## :page_with_curl: License
[MIT](https://choosealicense.com/licenses/mit/)

## :woman_student: :man_student: Credits 
  - Kornelia Maik
  - Radosław Leszkiewicz
  
_The project was conducted during the Microprocessor Lab course held by the
Institute of Control and Information Engineering, Poznan University of Technology.
Supervisor: Adam Bondyra_
