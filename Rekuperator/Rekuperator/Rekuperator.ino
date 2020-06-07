
// Attach a rotary encoder with output pins to A2 and A3.
// The common contact should be attached to ground.

/*
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin A0
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin A1
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/

// include the library code:
#include <LiquidCrystal.h>
#include <RotaryEncoder.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

#define ONE_WIRE_BUS 8
#define pin_fan_rotate 3
#define pin_relay 6
#define pin_pwm_fan_out 5
#define pin_pwm_fan_in 9
#define key_pressed 3
#define key_holded 4
#define backlight_lcd 10
#define pin_DS18B20_power 13
#define mode_rekuperator 1
#define mode_ventilation_in 2
#define mode_ventilation_out 3
#define mode_ventilation_pulse_in 4
#define mode_ventilation_pulse_out 5
#define mode_off 6
#define indication_main_screen 0
#define indication_menu_1 1
#define indication_menu_2 2
#define indication_menu_3 3
#define indication_select_mode 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer = { 0x28, 0xFF, 0x53, 0x52, 0x84, 0x16, 0x05, 0xC9 };
DeviceAddress outsideThermometer   = { 0x28, 0xFF, 0x51, 0x56, 0x84, 0x16, 0x05, 0x44 };
//DeviceAddress insideThermometer, outsideThermometer;
//Device 0 Address: 28FF515684160544
//Device 1 Address: 28FF5352841605C9

// данные настройки, изменяемые пользователем
uint8_t backlight_timeout = 25; // таймаут подсветки, секунды
const uint8_t menu_timeout = 17; // таймаут нахождения в меню при неактивности, секунды
const uint16_t lcd_refresh = 500; // частота обновлений экрана, миллисекунд
uint16_t rekuperator_in_time = 10; // время притока в режиме рекуперации, секунды
uint8_t rekuperator_in_out_time = 5; // пауза между притоком и вытяжкой в режиме рекуперации, секунды
uint16_t rekuperator_out_time = 10; // время вытяжки в режиме рекуперации, секунды
uint8_t rekuperator_out_in_time = 5; // пауза между вытяжкой и притоком в режиме рекуперации, секунды
uint16_t ventilation_pulse_in_time = 50; // время притока в режиме прерывистой вентиляции, секунды
uint16_t ventilation_pause_in_time = 15; // время паузы в режиме прерывистой вентиляции, секунды
uint16_t ventilation_pulse_out_time = 50; // время вытяжки в режиме прерывистой вентиляции, секунды
uint16_t ventilation_pause_out_time = 15; // время вытяжки в режиме прерывистой вентиляции, секунды


// служебные переменные (не трогать)
uint8_t lcd_blink = 0;
boolean lcd_refresh_allow = 1; // при выставлении в 1 происходит отрисовка, выставляется софт таймером
uint8_t work_mode = 1; // переменная выбора текущего режима работы
uint8_t indication_mode = 0; // переменная текущего режима индикации
uint8_t current_stage = 1; // переменная текущего шага внутри режима
volatile uint8_t rpp = 0; // для подсчёта оборотов вентилятора по прерыванию
uint8_t key_data = 0; // для работы функции считывания состояния кнопки
uint32_t millis_fan = 0;
uint32_t millis_rekuperation_ventilation_time = 0; // для отсчёта времени в режимах работы вентиляторов
uint32_t millis_backlight = 0; // для таймаута подсветки
uint32_t millis_menu_timeout = 0; // для таймаута меню
uint32_t millis_temp_scan = 0; // для периода опроса датчиков температуры
uint32_t millis_fan_rotate_scan = 0;
volatile uint32_t fan_impulse_period_read = 0;
volatile uint32_t micros_impulse_diff = 0;
uint8_t fan_scan_stop = 1;
float temp_in = 0;
float temp_out = 0;
uint8_t fan_speed = 0;
uint8_t fan_rpp = 0;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
//const int rs = 12, en = 11, d4 = 14, d5 = 4, d6 = 15, d7 = 2;
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

LiquidCrystal lcd(12, 11, 14, 4, 15, 2);

const byte House[8] = {
  B10000,
  B11000,
  B11100,
  B11110,
  B00111,
  B00110,
  B11110,
  B11110
};

const byte rus_p[8] = // П - матрица
{
  B11111,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B00000,
};

const byte rus_ya[8] = // Я - матрица
{
  B01111,
  B10001,
  B10001,
  B01111,
  B00101,
  B01001,
  B10001,
  B00000,
};

const byte rus_l[8] = // Л - матрица
{
  B00111,
  B01001,
  B01001,
  B01001,
  B01001,
  B01001,
  B10001,
  B00000,
};

const byte rus_i[8] = // И - матрица
{
  B10001,
  B10001,
  B10011,
  B10101,
  B11001,
  B10001,
  B10001,
  B00000,
};

const byte rus_c[8] = // Ц - матрица
{
  B10010,
  B10010,
  B10010,
  B10010,
  B10010,
  B10010,
  B11111,
  B00001,
};

byte rus_yy[8] = // Ы - матрица
{
  B10001,
  B10001,
  B10001,
  B11101,
  B10011,
  B10011,
  B11101,
  B00000,
};

byte rus_zg[8] = // Ж - матрица
{
  B10101,
  B10101,
  B01110,
  B00100,
  B01110,
  B10101,
  B10101,
  B00000,
};

/*
const byte rus_y[8] = // У - матрица
{ B10001,
  B10001,
  B01001,
  B00111,
  B00001,
  B00010,
  B01100,
  B00000,
};
*/

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(A2, A3);

void setup()
{
attachInterrupt(1, rotate_counter, RISING);
sensors.begin();
sensors.setResolution(10); // 9-12
pinMode(pin_fan_rotate, INPUT); //
pinMode(pin_relay, OUTPUT); //
pinMode(pin_pwm_fan_in, OUTPUT); //
pinMode(pin_pwm_fan_out, OUTPUT); //
pinMode(backlight_lcd, OUTPUT);
pinMode(LED_BUILTIN, OUTPUT); //
digitalWrite(pin_relay,HIGH);
digitalWrite(backlight_lcd, HIGH);
digitalWrite(pin_pwm_fan_in,LOW);
digitalWrite(pin_pwm_fan_out,LOW);
digitalWrite(LED_BUILTIN,LOW); // подаём питание на DS18B20
lcd.createChar(0, House);
lcd.createChar(1, rus_p);
lcd.createChar(2, rus_ya);
lcd.createChar(3, rus_l);
lcd.createChar(4, rus_i);
lcd.createChar(5, rus_c);
lcd.createChar(6, rus_yy);
lcd.createChar(7, rus_zg);
lcd.begin(16, 2); // инициализация дисплея
lcd.clear(); // очистка дисплея
//lcd.print("hello, world!");
}


void loop()
{
 key_data = get_key();  // забираем значение состояния кнопки
 static int pos = 0; // переменная состояния энкодера
 encoder.tick(); // запуск проверки состояния энкодера
 int newPos = encoder.getPosition(); // считывание текущего состояния энкодера
 lcd_refresh_timer(); // таймер обновления экрана
 
 {//---------------------управление подсветкой------------------------------ 
 if ((backlight_timeout != 0) || (backlight_timeout < 100))
{
 if ((pos != newPos))
  {
    if (!digitalRead(backlight_lcd))
	{
		digitalWrite(backlight_lcd, HIGH);
		millis_backlight = millis();
		pos = newPos;
	}
	else 
	{
		millis_backlight = millis();
	}
  }
 else if ((key_data))
  {
	if (!digitalRead(backlight_lcd))
	{
		digitalWrite(backlight_lcd, HIGH);
		millis_backlight = millis();
		key_data = 0;
	}
	else 
	{
		millis_backlight = millis();
	}	
  }
 if (digitalRead(backlight_lcd)) 
  {
	if ((millis() - millis_backlight) > (backlight_timeout*1000))
	{
		digitalWrite(backlight_lcd, LOW);		
	}
  }
}
else if ((backlight_timeout == 0) && (digitalRead(backlight_lcd)))
	{
		digitalWrite(backlight_lcd, 0);
	}
else if ((backlight_timeout > 99) && (!digitalRead(backlight_lcd)))
	{
		digitalWrite(backlight_lcd, 1);
	}
}//------------------------------------------------------------------------------

{//-----------------------выход меню выбора режимов------------------------------
if (key_data == key_pressed)
	{
		if (indication_mode == indication_main_screen)
		{
			indication_mode = indication_select_mode;
			key_data = 0;
			millis_menu_timeout = millis();
      pos = newPos;
		}
		else if (indication_mode == indication_select_mode)
		{
			indication_mode = indication_main_screen;
			key_data = 0;
		}
	}
}
//-------------------------------------------------------------------------------
{//----------------------меню выбора режимов-------------------------------------
if (indication_mode == indication_select_mode)
	{
//if (pos != newPos) 
  if (((pos - 3) >= newPos) || ((pos + 3) <= newPos))
	{
    if ((pos > newPos) && (work_mode < 6))
      {
        work_mode++;
        current_stage = 1;
        digitalWrite(pin_pwm_fan_in, LOW);
        digitalWrite(pin_pwm_fan_out, LOW);
      }
    else if ((pos < newPos) && (work_mode > 1))
      {
        work_mode--;
        current_stage = 1;
        digitalWrite(pin_pwm_fan_in, LOW);
        digitalWrite(pin_pwm_fan_out, LOW);
      }
		pos = newPos;
		millis_menu_timeout = millis();
		lcd.clear();
	}
		if ((millis()-millis_menu_timeout) > (menu_timeout * 1000))
		{
			indication_mode = indication_main_screen;
		}
	}
}
switch (work_mode)
{
	case mode_rekuperator:
	{
		if (lcd_refresh_allow)
			{
				if ((indication_mode == indication_main_screen) || ((indication_mode == indication_select_mode) && !lcd_blink))
				{
					lcd.setCursor(2, 0);
					lcd.print("PEK\xBF""\01""EPA\05\04\02");
				}
				else 
			    {
					lcd.setCursor(0, 0);
					lcd.print("                ");
			    }
				lcd.setCursor(0, 1);
				lcd.print(temp_in);
				lcd.setCursor(11, 1);
				lcd.print(temp_out);
				lcd.setCursor(7, 1);
				lcd.write(byte(0));
			}
			if (current_stage == 1)
			{
				if (lcd_refresh_allow)
					{
							lcd.setCursor(8, 1);
							lcd.print(">"); 							
					}
				if (!digitalRead(pin_pwm_fan_out))
					{
						digitalWrite(pin_pwm_fan_out, HIGH);
						digitalWrite(pin_pwm_fan_in, LOW);
						millis_rekuperation_ventilation_time = millis();
					}
				else if ((millis() - millis_rekuperation_ventilation_time) > (rekuperator_out_time * 1000))
					{
						digitalWrite(pin_pwm_fan_out, LOW);
						digitalWrite(pin_pwm_fan_in, LOW);
						millis_rekuperation_ventilation_time = millis();
						current_stage = 2;
					}
			}
			else if (current_stage == 2)
			{
				if (lcd_refresh_allow)
					{
							lcd.setCursor(8, 1);
							lcd.print("|"); 							
					}
			if ((millis() - millis_rekuperation_ventilation_time) > (rekuperator_out_in_time * 1000))
					{
						millis_rekuperation_ventilation_time = millis();
						current_stage = 3;
					}
			}
			else if (current_stage == 3)
			{
				if (lcd_refresh_allow)
					{
							lcd.setCursor(8, 1);
							lcd.print("<"); 							
					}
				if (!digitalRead(pin_pwm_fan_in))
					{
						digitalWrite(pin_pwm_fan_in, HIGH);
						digitalWrite(pin_pwm_fan_out, LOW);
						millis_rekuperation_ventilation_time = millis();
					}
				else if ((millis() - millis_rekuperation_ventilation_time) > (rekuperator_in_time * 1000))
					{
						digitalWrite(pin_pwm_fan_in, LOW);
						digitalWrite(pin_pwm_fan_out, LOW);
						millis_rekuperation_ventilation_time = millis();
						current_stage = 4;
					}			
			}
			else if (current_stage == 4)
			{
				if (lcd_refresh_allow)
					{             
							lcd.setCursor(8, 1);
							lcd.print("|"); 							
					}
			if ((millis() - millis_rekuperation_ventilation_time) > (rekuperator_in_out_time * 1000))
					{
						millis_rekuperation_ventilation_time = millis();
						current_stage = 1;
					}
			}

		}
		
	break;
	
	case mode_ventilation_in:
{
  digitalWrite(pin_pwm_fan_in, HIGH);
  digitalWrite(pin_pwm_fan_out, LOW);
        if (lcd_refresh_allow)
          {	  if ((indication_mode == indication_main_screen) || ((indication_mode == indication_select_mode) && !lcd_blink))
			{
				lcd.setCursor(0, 0);
				lcd.print("     \01P\04TOK    "); 
			}
			else 
			{
				lcd.setCursor(0, 0);
				lcd.print("                ");
			}
              lcd.setCursor(0, 1);
              lcd.print(temp_in);
              lcd.setCursor(11, 1);
              lcd.print(temp_out);
              lcd.setCursor(7, 1);
              lcd.write(byte(0));
              lcd.print("<");               
          }
}	
	break;
	
	case mode_ventilation_out:
{
  digitalWrite(pin_pwm_fan_in, LOW);
  digitalWrite(pin_pwm_fan_out, HIGH);
        if (lcd_refresh_allow)
          {	  
		    if ((indication_mode == indication_main_screen) || ((indication_mode == indication_select_mode) && !lcd_blink))
			{
              lcd.setCursor(0, 0);
              lcd.print("    B\x06T\x02\x07KA"   );
			}
       else 
        {
              lcd.setCursor(0, 0);
              lcd.print("                ");
        }      
              lcd.setCursor(0, 1);
              lcd.print(temp_in);
              lcd.setCursor(11, 1);
              lcd.print(temp_out);
              lcd.setCursor(7, 1);
              lcd.write(byte(0));
              lcd.print(">");               
          }
}  	
	break;	
	
	case mode_ventilation_pulse_in:
	{
		if (lcd_refresh_allow)
          {	  
		    if ((indication_mode == indication_main_screen) || ((indication_mode == indication_select_mode) && !lcd_blink))
			{
				lcd.setCursor(0, 0);
				lcd.print("  \01P\04TOK \05\04K\03.  ");   
			}
			else 
			{
			  lcd.setCursor(0, 0);
			  lcd.print("                ");
			}
              lcd.setCursor(0, 1);
              lcd.print(temp_in);
              lcd.setCursor(11, 1);
              lcd.print(temp_out);
              lcd.setCursor(7, 1);
              lcd.write(byte(0));         
          }
			if (current_stage == 1)
			{
				if (lcd_refresh_allow)
					{
							lcd.setCursor(8, 1);
							lcd.print("<"); 							
					}
				if (!digitalRead(pin_pwm_fan_in))
					{
						digitalWrite(pin_pwm_fan_in, HIGH);
						digitalWrite(pin_pwm_fan_out, LOW);
						millis_rekuperation_ventilation_time = millis();
					}
				else if ((millis() - millis_rekuperation_ventilation_time) > (ventilation_pulse_in_time* 1000))
					{
						digitalWrite(pin_pwm_fan_in, LOW);
						digitalWrite(pin_pwm_fan_out, LOW);
						millis_rekuperation_ventilation_time = millis();
						current_stage = 2;
					}
			}
			else if (current_stage == 2)
			{
				if (lcd_refresh_allow)
					{
							lcd.setCursor(8, 1);
							lcd.print("|"); 							
					}
			if ((millis() - millis_rekuperation_ventilation_time) > (ventilation_pause_in_time * 1000))
					{
						millis_rekuperation_ventilation_time = millis();
						current_stage = 1;
					}
			}
	}
	
	break;	
	
	case mode_ventilation_pulse_out:
	
{	
        if (lcd_refresh_allow)
          {
			  if ((indication_mode == indication_main_screen) || ((indication_mode == indication_select_mode) && !lcd_blink))
			  {
				lcd.setCursor(0, 0);
				lcd.print("  B\x06T\x02\x07KA \05\04K\03. ");  
			  }
			  else 
			  {
			  lcd.setCursor(0, 0);
			  lcd.print("                ");
			  }
              lcd.setCursor(0, 1);
              lcd.print(temp_in);
              lcd.setCursor(11, 1);
              lcd.print(temp_out);
              lcd.setCursor(7, 1);
              lcd.write(byte(0));			  
          }
			if (current_stage == 1)
			{
				if (lcd_refresh_allow)
					{
							lcd.setCursor(8, 1);
							lcd.print(">"); 							
					}
				if (!digitalRead(pin_pwm_fan_out))
					{
						digitalWrite(pin_pwm_fan_in, LOW);
						digitalWrite(pin_pwm_fan_out, HIGH);
						millis_rekuperation_ventilation_time = millis();
					}
				else if ((millis() - millis_rekuperation_ventilation_time) > (ventilation_pulse_out_time* 1000))
					{
						digitalWrite(pin_pwm_fan_in, LOW);
						digitalWrite(pin_pwm_fan_out, LOW);
						millis_rekuperation_ventilation_time = millis();
						current_stage = 2;
					}
			}
			else if (current_stage == 2)
			{
				if (lcd_refresh_allow)
					{
							lcd.setCursor(8, 1);
							lcd.print("|"); 							
					}
			if ((millis() - millis_rekuperation_ventilation_time) > (ventilation_pause_out_time * 1000))
					{
						millis_rekuperation_ventilation_time = millis();
						current_stage = 1;
					}
			}
}
	break;	

  case mode_off:
   {
      digitalWrite(pin_pwm_fan_in, LOW);
      digitalWrite(pin_pwm_fan_out, LOW);
      digitalWrite(pin_relay, HIGH);
              if (lcd_refresh_allow)
          {
			  if ((indication_mode == indication_main_screen) || ((indication_mode == indication_select_mode) && !lcd_blink))
			  {
				lcd.setCursor(0, 0);
				lcd.print("  STANDBY MODE  ");       
			  }
			  else 
			  {
			  lcd.setCursor(0, 0);
			  lcd.print("                ");
			  }
              lcd.setCursor(0, 1);
              lcd.print(temp_in);
              lcd.setCursor(11, 1);
              lcd.print(temp_out);
              lcd.setCursor(7, 1);
              lcd.write(byte(0));
              lcd.print("|");               
          }
    
   }

}


/*

    if (pos != newPos) 
  {
      if ((pos > newPos) && fan_rpp < 250)
      {
        fan_rpp++;
      }
      else if ((pos < newPos) && fan_rpp > 0)
      {
        fan_rpp--;        
      }
//    Serial.print(newPos);
//    Serial.println();
    pos = newPos;
  } 
  
 
  
  
 	if ((millis() - millis_fan_rotate_scan) > 150)
	{
	uint32_t fan_impulse_period_read_temp = 0;
	if (fan_scan_stop == 2)
	{
	fan_scan_stop = 0;
	fan_impulse_period_read_temp = fan_impulse_period_read;
	fan_scan_stop = 1;
	}	

  analogWrite(pin_pwm_fan_out, fan_rpp); //0-255  
  analogWrite(pin_pwm_fan_in, fan_rpp); //0-255 

	lcd.setCursor(0, 0);
	lcd.print("in: "); 
	lcd.print(temp_in); 	
	lcd.setCursor(0, 1);
	lcd.print("out: ");	
	lcd.print(temp_out);


	
//  lcd.print(fan_speed); 
//  lcd.print("  ");  
//	lcd.setCursor(10, 0);
//	lcd.print(fan_impulse_period_read);		
//  lcd.print("  ");   
//  lcd.setCursor(11, 1);
//  lcd.print(fan_rpp);     
//  lcd.print("  ");  
  
	}
	
	*/

	
	

/*
  if (key_data == key_holded) //
{
digitalWrite(LED_BUILTIN,HIGH);
digitalWrite(pin_relay,HIGH);
analogWrite(pin_pwm_fan_in, fan_speed); //0-255
//digitalWrite(pin_pwm_fan_in,HIGH);
digitalWrite(pin_pwm_fan_out,LOW);
//work_millis = millis();
//ozone_millis = millis();
key_data = 0; //
}

  if (key_data == key_pressed) // 
{
digitalWrite(LED_BUILTIN,LOW);
digitalWrite(pin_relay,LOW);
digitalWrite(pin_pwm_fan_in,LOW);
analogWrite(pin_pwm_fan_out, fan_speed); //0-255
//digitalWrite(pin_pwm_fan_out,HIGH);
//work_millis = millis();
//ozone_millis = 0;
key_data = 0; //
}
*/
if ((millis() - millis_temp_scan) > 1000)
  {
  sensors.requestTemperatures();
  temp_in = sensors.getTempC(insideThermometer);
  temp_out = sensors.getTempC(outsideThermometer);
  millis_temp_scan = millis();
  }
if (indication_mode == indication_main_screen) // если находимся в основном режиме - сбрасываем показания энкодера для корректной работы таймаута подсветки
	{
		pos = newPos;
	}
key_data = 0; //
  } // loop ()

byte get_key()
{
uint8_t trigger_push_hold_counter = 10; 
uint8_t milliseconds_between_increment = 50; 
static uint8_t  val_ke;
static uint32_t key_delay_millis;
static uint32_t key_delay_after_hold_millis;
if ((millis() - key_delay_millis) > milliseconds_between_increment)
{
  if (!(PIND & (1 << PIND7)))
    {
    val_ke++;
    if (val_ke > trigger_push_hold_counter)
        {
         val_ke = 0;
         key_delay_after_hold_millis = millis();
         return key_holded;
        }   
    }
  key_delay_millis = millis();
}
if (val_ke > 0)
{
  if ((PIND & (1 << PIND7)) && ((millis() - key_delay_after_hold_millis) > (trigger_push_hold_counter * milliseconds_between_increment)))
  {
  val_ke = 0;
  return key_pressed;
  }
}
if (PIND & (1 << PIND7)) {val_ke = 0;}
return 0;
}

void lcd_refresh_timer()
{
	static uint32_t millis_lcd_refresh = 0; // для интервалов обновления экрана
	if (lcd_refresh_allow == 1)
	{
		lcd_refresh_allow = 0;
	}
	if ((millis()-millis_lcd_refresh) > lcd_refresh)
	{
		lcd_refresh_allow = 1;
    lcd_blink++;
    if (lcd_blink > 1) {lcd_blink = 0;}
    millis_lcd_refresh = millis();
	}
}
void rotate_counter()
{
    if(fan_scan_stop == 1) 
	{
	fan_impulse_period_read =micros()-micros_impulse_diff;
	fan_scan_stop = 2;
	}
    micros_impulse_diff = micros();
}
