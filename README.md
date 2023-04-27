					    		ЕLЕKТRОТЕHNIČKI FАKULТЕТ
						    UNIVЕRZIТЕТ U ISТОČNОM SАRАЈЕVU

					PRAĆENJE PARAMETARA ELEKTROMOTORA POMOĆU KONCEPTA IIoT
						
							      ZAVRŠNI RAD

				     Kandidat: Boško Vasić, 1946	Меntоr: doc. dr Nataša Popović

						     Istоčnо Sаrајеvо, april 2023.

*****************************************************************************************************************************************************************************************
U ovom projektu je demonstrirana primjena ESP32  mikrokontrolera kako bi se prikazala upotreba koncepta IIoT u industrijskim aplikacijama na primjeru praćenja parametara elektromotora 
(napon, struja, temperatura i brzina). Ovi parametri se prikupljaju sa predviđenih senzora i prikazuju na lokalnom ekranu na eksperimentalnoj ploči, veb stranici pomoću usluge 
ThingSpeak  platforme za IoT analitiku i na mobilnom telefonu pomoću aplikacije ThingView - ThingSpeak viewer. 

*****************************************************************************************************************************************************************************************
						      ****** VAŽNE NAPOMENE *****

U okruženju Arduino IDE 1.8.13 nakon odabira ESP32 ploče i instalacije potrebnih biblioteka, prilikom kompajlovanja pod menijem Tools, Arduino Runs On: Odabrati Core 0 !!! 
*****Jer u kodu imamo definisanu funkciju koja će raditi ThingSpeak razmjenu podataka na Core 1 *****

U kodu izmjeniti parametre za pristup Wi-Fi mreži kao i ThingSpeak kanal, API ključeve i podesiti polja prema podešavanjima na ThingSpeak platformi.

Primjeri API zahtjeva za izmjenu režima rada i brzine obrtanja putem ThingSpeak platforme:
https://api.thingspeak.com/update.json?api_key=IL62361NNH9W5X53&field3=2
https://api.thingspeak.com/update.json?api_key=IL62361NNH9W5X53&field3=1
https://api.thingspeak.com/update.json?api_key=IL62361NNH9W5X53&field4=500

*****************************************************************************************************************************************************************************************




