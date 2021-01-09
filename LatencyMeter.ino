#include <Nextion.h>

#define LEVEL_LOW   (0)
#define CH0_OFFSET  (25) //Normal shaft height
#define NOTE  1000 //Tint set to buzzer

#define BIT12MAX 4095
#define CHART_MAX 50
#define MAIN_MAX 40

//const int PhotoDiode_vout_pin = 2;
const int PhotoDiode_pin  = 15;

//const int Audio_PD_vout_pin = 5;
const int Audio_PD_pin  = 13;

const int Audio1_pin = 2;
const int Audio2_pin = 4;

uint32_t th1_value = 3200;
uint32_t th2_value = 1000;
const int audio_th = 1100;

uint32_t pixel_delay = 10000; //us

bool mode_VA = true;
bool triggered = false;
bool refractory = false;
bool video_on = false;
bool audio_on = false;
bool alternator = false;

unsigned long trigger_time = 0;
const unsigned long timeout = 800000;
const unsigned long ref_period = 200000; //after result no new trigger

int buzz=13;                   //piezo buzzer to digital pin 13  
int senRead=0;                 //Readings from sensor to analog pin 0  
int limit=850;                 //Threshold range of an obstacle  
int step=0;

unsigned long last_pixel = 0;
unsigned long last_scale = 0;
unsigned long micros_now_reader = 0;
unsigned long micros_now_pixel = 0;
unsigned long result = 0;
unsigned long result_time = 0;
unsigned long last_video_on = 0;
unsigned long last_audio_on = 0;

float measured_pixel_s = pixel_delay;
const float measured_pixel_coef = 0.95;

NexWaveform video = NexWaveform(0,  1, "video");
NexWaveform audio = NexWaveform(0, 10, "audio");
NexWaveform main  = NexWaveform(0,  9, "main");
NexSlider th1 = NexSlider(0, 5, "th1");
NexSlider th2 = NexSlider(0, 6, "th2");
NexRadio opt_VA = NexRadio(0, 11, "opt_VA");
NexRadio opt_AV = NexRadio(0, 12, "opt_AV");
NexNumber pDelay = NexNumber(0, 8, "pDelay");
NexNumber r1 = NexNumber(0, 2, "r1");
NexNumber r2 = NexNumber(0, 3, "r2");
NexNumber r3 = NexNumber(0, 4, "r3");
NexNumber measured = NexNumber(0, 17 , "measured");


bool video_pixel_just_written = true;
bool audio_pixel_just_written = true;

int PD_val        =    0;
int PD_val_pixel  =    0;

int Audio_PD_val        = 4095;
int Audio_PD_val_pixel  = 4095;

int Audio1_val        = 4095;
int Audio1_val_pixel  = 4095;

int Audio2_val        = 4095;
int Audio2_val_pixel  = 4095;

NexTouch *nex_listen_list[] = {
    &th1,
    &th2,
    &opt_VA,
    &opt_AV,
    &pDelay,
    NULL
};

void Pixel( void * pvParameters ){
  while (true) {
    
    micros_now_pixel = micros();
    int measured_pixel = micros_now_pixel - last_pixel;

    if (measured_pixel > pixel_delay) {
      if (last_pixel<result_time)
        update_result(result);

       
      video.addValue(0,
        map(PD_val_pixel,       0, BIT12MAX, 0+2, CHART_MAX-2));
      video.addValue(1,
        map(th1_value,          0, BIT12MAX, 0+2, CHART_MAX-2));
      audio.addValue(0,
        map(Audio1_val_pixel,   0, BIT12MAX, 0+2, CHART_MAX-2));
      audio.addValue(1,
        map(Audio2_val_pixel,   0, BIT12MAX, 0+2, CHART_MAX-2));
      audio.addValue(2,
        map(Audio_PD_val_pixel, 0, BIT12MAX, 0+2, CHART_MAX-2));
      audio.addValue(3,
        map(th2_value,          0, BIT12MAX, 0+2, CHART_MAX-2));

      bool pixel_video_on = video_on || (last_pixel<last_video_on);
      bool pixel_audio_on = audio_on || (last_pixel<last_audio_on);
        
      last_pixel = micros_now_pixel;

      alternator = !alternator;
      int rc = (10 * (micros_now_pixel - trigger_time)) / timeout; //result clock
      main.addValue(0, 
          20 +
          alternator *
          (
              18 * pixel_video_on +
              !pixel_video_on *
              (
                rc * (!pixel_audio_on &&  triggered) +
               -rc * ( pixel_audio_on &&  triggered)
              )
          ) +
          !alternator *
          (
              -18 * pixel_audio_on +
              !pixel_audio_on *
              (
               -rc * (!pixel_video_on &&  triggered) +
                rc * ( pixel_video_on &&  triggered)
              )
             
             
          )
         );
      
      /*
          0 +
         (alternator || !triggered) *
         (
            15 * pixel_audio_on +
            22 * pixel_video_on
         ) +
         (!alternator && triggered) *
         (
            (8 * (micros_now_pixel - trigger_time)) / timeout
         )
         );
         
        */ 
       /*   
        
          7 +
         -1 * triggered*(!pixel_video_on)*(!pixel_audio_on)*(2+4*alternator) +
         10 * pixel_audio_on +
         20 * pixel_video_on
        );*/
      video_pixel_just_written = true;
      audio_pixel_just_written = true;

      if (step>(5000000.0/pixel_delay))
      {
        measured.setValue(measured_pixel);
        step=0;
      }
      step++;
      
    }
    //else update for maximum in boolean
    
    nexLoop(nex_listen_list);
    
//Serial.printf ("$%d %d %d %d %d %d;", PD_val, video_on, Audio_PD_val, Audio1_val, Audio2_val, audio_on);
        //Serial.printf ("%d %d %d %d %d %d", PD_val, video_on, Audio_PD_val, Audio1_val, Audio2_val, audio_on);
        //Serial.println(" ");
delay(1);
//delayMicroseconds(500);
  }
}

void Reader( void * pvParameters ){
     while(true){
      
        micros_now_reader = micros();
        PD_val=analogRead(PhotoDiode_pin);  //variable to store values from the photodiode  
        micros_now_reader = (micros() + micros_now_reader)/2;

        refractory = refractory && ((micros_now_reader - result_time)<ref_period);
        
        video_on = PD_val>th1_value;
        if (video_on)
          last_video_on = micros_now_reader;
        
        if (mode_VA) {
          if (video_on && !triggered && !refractory) {
            triggered = true;
            trigger_time = micros_now_reader;
          }
          else {
            triggered = triggered && ((micros_now_reader - trigger_time)<timeout);
          }
        }
        else {
          if (video_on) {
            if (triggered) {
              result = micros_now_reader-trigger_time;
              result_time = micros();
              triggered = false;
              refractory = true;
            }
            else {
              //
            }
          }
        }
        
        
        micros_now_reader = micros();
        Audio_PD_val=analogRead(Audio_PD_pin);
        Audio1_val=analogRead(Audio1_pin) * 2; /////
        Audio2_val=analogRead(Audio2_pin);
        micros_now_reader = (micros() + micros_now_reader)/2;

        refractory = refractory && ((micros_now_reader - result_time)<ref_period);
        
        audio_on = (Audio1_val<audio_th) || (Audio2_val<audio_th) || (Audio_PD_val<th2_value);
        if (audio_on)
          last_audio_on = micros_now_reader;
        
        if (!mode_VA) {
          if (audio_on && !triggered && !refractory) {
            triggered = true;
            trigger_time = micros_now_reader;
          }
          else {
            triggered = triggered && ((micros_now_reader - trigger_time)<timeout);
          }
        }
        else {
          if (audio_on) {
            if (triggered) {
              result = micros_now_reader-trigger_time;
              result_time = micros();
              triggered = false;
              refractory = true;
            }
            else {
              //
            }
          }
        }

    PD_val_pixel        = PD_val       * video_pixel_just_written + !video_pixel_just_written * max(PD_val_pixel, PD_val);
    Audio_PD_val_pixel  = Audio_PD_val * audio_pixel_just_written + !audio_pixel_just_written * min(Audio_PD_val_pixel, Audio_PD_val);
    Audio1_val_pixel    = Audio1_val   * audio_pixel_just_written + !audio_pixel_just_written * min(Audio1_val_pixel, Audio1_val);
    Audio2_val_pixel    = Audio2_val   * audio_pixel_just_written + !audio_pixel_just_written * min(Audio2_val_pixel, Audio2_val);
    video_pixel_just_written = false;
    audio_pixel_just_written = false;


    //delayMicroseconds(500);
    delay(1);
    
    } 
}


void th1Callback(void *ptr) {
    Serial.println("th1Callback");
    th1.getValue(&th1_value);
}

void th2Callback(void *ptr) {
    Serial.println("th2Callback");
    th2.getValue(&th2_value);
}

void opt_VACallback(void *ptr) {
    Serial.println("opt_VACallback");
    mode_VA = true;
    triggered = false;
}

void opt_AVCallback(void *ptr) {
    Serial.println("opt_AVCallback");
    mode_VA = false;
    triggered = false;
}

void pDelayCallback(void *ptr) {
    Serial.println("pDelayCallback");
    pDelay.getValue(&pixel_delay);
}

void update_result(unsigned long result) {
  uint32_t tmp;
  r2.getValue(&tmp);
  r3.setValue(tmp);
  r1.getValue(&tmp);
  r2.setValue(tmp);
  r1.setValue(result);
}

void setup()    
{  
  Serial.begin(115200);
  Serial.println("Device Start!");

  nexInit(); //Initializes communication with the Nextion Display.


  th1.attachPop(th1Callback, &th1);
  th2.attachPop(th2Callback, &th2);
  opt_VA.attachPop(opt_VACallback, &opt_VA);
  opt_AV.attachPop(opt_AVCallback, &opt_AV);
  pDelay.attachPop(pDelayCallback, &pDelay);

  Serial.println("Start Pixel");
  xTaskCreatePinnedToCore(Pixel, "Pixel", 10000, NULL, 2, NULL, 1);
  delay(500);
  Serial.println("Start Reader");
  xTaskCreatePinnedToCore(Reader, "Reader", 10000, NULL, 2, NULL, 0);
}




void loop()  
{  
}
