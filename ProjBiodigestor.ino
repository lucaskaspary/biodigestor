#include <Thermistor.h>
#include <LiquidCrystal.h>

#define HEATING_MIN_TEMP 35
#define HEATING_MAX_TEMP 40

#define INTERNAL_MIN_TEMP 28
#define INTERNAL_MAX_TEMP 32

#define PIN_BOMB  6
#define PIN_HOT   7
#define PIN_COMP  8


float internal_temp;
float heating_temp;

class BiodigestorPins {
  public:
    BiodigestorPins() {};
    ~BiodigestorPins() {};
    
    int thermistor_internal;
    int thermistor_heating;
    
    int lcd_rs;
    int lcd_enable;
    int lcd_d4;
    int lcd_d5;
    int lcd_d6;
    int lcd_d7;
    
    int relay_water_pump;
    int relay_heater;
    int relay_compressor;
    int relay_aux;
    
    int sensor_gas_presence;
    int sensor_gas_pressure;
};

class Biodigestor {
  public:
    Biodigestor(BiodigestorPins* pins);
    ~Biodigestor(){};

    void ShowStatus();
    void ProcessSensors();
    void ProcessHeatingSystem();
    void ProcessCompressorSystem();
    
  private:
    void Setup();
    BiodigestorPins pins_;
    
    LiquidCrystal* lcd_;
    Thermistor* thermistor_insternal_;
    Thermistor* thermistor_heating_;

    bool heating_enable_;
    bool water_pump_enable_;
    bool compressor_enable_;

    bool sensor_gas_presence_;
    bool sensor_gas_pressure_;

    float temp_heating_;
    float temp_internal_;
    
};

Biodigestor::Biodigestor(BiodigestorPins* pins)
: lcd_(nullptr), thermistor_insternal_(nullptr), thermistor_heating_ (nullptr), heating_enable_(false), water_pump_enable_(false)
{
  memcpy(&pins_, pins, sizeof(BiodigestorPins));
  Setup();
}

void Biodigestor::Setup() {
  
  lcd_ = new LiquidCrystal(pins_.lcd_rs, pins_.lcd_enable, pins_.lcd_d4, pins_.lcd_d5, pins_.lcd_d6, pins_.lcd_d7);
  lcd_->begin(20, 4);

  thermistor_insternal_ = new Thermistor(pins_.thermistor_internal);
  thermistor_heating_ = new Thermistor(pins_.thermistor_heating);

  pinMode(pins_.relay_water_pump, OUTPUT);
  digitalWrite(pins_.relay_water_pump,HIGH);
  
  pinMode(pins_.relay_heater, OUTPUT);
  digitalWrite(pins_.relay_heater,HIGH);
  
  pinMode(pins_.relay_compressor, OUTPUT);
  digitalWrite(pins_.relay_compressor,HIGH);
  
  pinMode(pins_.relay_aux, OUTPUT);
  digitalWrite(pins_.relay_aux,HIGH); 
}

void Biodigestor::ShowStatus() {
  lcd_->clear();
  
  lcd_->setCursor(4,0);
  lcd_->print("BIODIGESTOR");
  lcd_->setCursor(0,1);
  lcd_->print("TI:");
  lcd_->setCursor(3,1);
  lcd_->print(temp_internal_);
  
  lcd_->setCursor(8,1);
  lcd_->print("C  TA:");
  lcd_->setCursor(14,1);
  lcd_->print(temp_heating_);
  lcd_->setCursor(19,1);
  lcd_->print("C");

  lcd_->setCursor(0,2);
  lcd_->print("BOMBA:");
  lcd_->setCursor(6,2);
  lcd_->print(water_pump_enable_?"LIGA":"DESL");
  
  lcd_->setCursor(11,2);
  lcd_->print("AQU:");
  lcd_->setCursor(15,2);
  lcd_->print(heating_enable_?"LIGA":"DESL");

  if(compressor_enable_ == true) {
    lcd_->setCursor(0,3);
    lcd_->print("Bombeando gas...");
  } else {
    if (sensor_gas_presence_ == true)
    {
      lcd_->setCursor(0,3);
      lcd_->print("Acumulando gas...");
    } else {
      lcd_->setCursor(0,3);
      lcd_->print("Acumulador vazio...");
    }
  }
  
}


void Biodigestor::ProcessSensors() {

  temp_heating_ = thermistor_heating_->getTemp();
  temp_internal_ = thermistor_insternal_->getTemp();

  if(analogRead(pins_.sensor_gas_presence) > 1000) {
    sensor_gas_presence_ = true;
  } else {
    sensor_gas_presence_ = false;
    compressor_enable_ = false;
  }

  if(analogRead(pins_.sensor_gas_pressure) > 1000) {
    sensor_gas_pressure_ = true;
  } else {
    sensor_gas_pressure_ = false;
  }

}


void Biodigestor::ProcessHeatingSystem() {
  
  if( temp_internal_ < INTERNAL_MIN_TEMP) {
    water_pump_enable_ = true;
  } else {
    if(temp_internal_ >= INTERNAL_MAX_TEMP) {
      water_pump_enable_ = false;
    }
  }

  if( water_pump_enable_ == true) {
    if( temp_heating_ < HEATING_MIN_TEMP) {
      heating_enable_ = true;
    } else {
      if(temp_heating_ >= HEATING_MAX_TEMP) {
        heating_enable_ = false;
      }
    }
  } else {
    heating_enable_ = false;
  }


  
  if(water_pump_enable_ == true) {
    digitalWrite(pins_.relay_water_pump,LOW);
  } else {
    digitalWrite(pins_.relay_water_pump,HIGH);
  }

  if(heating_enable_ == true) {
    digitalWrite(pins_.relay_heater,LOW);
  } else {
    digitalWrite(pins_.relay_heater,HIGH);
  }
  
}

void Biodigestor::ProcessCompressorSystem() {
   if((sensor_gas_pressure_ == true) && (sensor_gas_presence_ == true)) {
    compressor_enable_ = true;
    digitalWrite(pins_.relay_compressor,LOW);
  } else {
    if((sensor_gas_pressure_ == false) && (sensor_gas_presence_ == false)) {
      compressor_enable_ = false;
      digitalWrite(pins_.relay_compressor,HIGH); 
    }
  }
}













Biodigestor* biodigestor;

void setup()
{
  Serial.begin(9600);
  
  BiodigestorPins biodigestorPins;
  biodigestorPins.thermistor_internal = A0;
  biodigestorPins.thermistor_heating = A1;

  biodigestorPins.lcd_rs = 12;
  biodigestorPins.lcd_enable = 11;
  biodigestorPins.lcd_d4 = 5;
  biodigestorPins.lcd_d5 = 4;
  biodigestorPins.lcd_d6 = 3;
  biodigestorPins.lcd_d7 = 2;

  biodigestorPins.relay_water_pump = 6;
  biodigestorPins.relay_heater = 7;
  biodigestorPins.relay_compressor = 8;
  biodigestorPins.relay_aux = 9;

  biodigestorPins.sensor_gas_presence = A2;
  biodigestorPins.sensor_gas_pressure = A3;

  biodigestor = new Biodigestor(&biodigestorPins);  
}



void loop() {
  
    biodigestor->ProcessSensors();
    biodigestor->ShowStatus();
    biodigestor->ProcessHeatingSystem();
    biodigestor->ProcessCompressorSystem();
    
    delay(1000);
}
